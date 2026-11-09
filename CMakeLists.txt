# ============================================================
# Project Definition
# ============================================================
cmake_minimum_required(VERSION 3.16)
project(Fx2DProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(TARGET_NAME Fx2D)

# Built on its own, Fx2D defaults to the full build: viewer, tests and the YAML loader. Added to
# another project with add_subdirectory() or FetchContent, it defaults to the headless physics
# library, without tests.
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(FX2D_IS_TOP_LEVEL ON)
  set(FX2D_HEADLESS_DEFAULT OFF)
else()
  set(FX2D_IS_TOP_LEVEL OFF)
  set(FX2D_HEADLESS_DEFAULT ON)
endif()

# Options
option(FX2D_WARNINGS "Enable extra warnings" ON)
option(FX2D_WERROR   "Treat warnings as errors" OFF)
option(FX2D_BUILD_TESTS "Build unit tests" ${FX2D_IS_TOP_LEVEL})

# Headless builds the physics library and tests without the viewer: no raylib, no Dear ImGui.
# Renderer.cpp, the viewer executable and the visual examples are left out.
option(FX2D_HEADLESS "Build without the viewer (no raylib/ImGui)" ${FX2D_HEADLESS_DEFAULT})

# The YAML scene loader (FxYAML) is the only part of the library that uses a third-party
# package. With it off, the library builds from the C++ standard library alone.
option(FX2D_YAML "Build the YAML scene loader (needs yaml-cpp)" ON)

# Anything needed but not installed is downloaded and built with the project, so a fresh clone
# configures without installing packages first. Turn this off to require installed packages
# instead, for distribution packaging or offline builds.
option(FX2D_FETCH_DEPS "Download yaml-cpp, raylib, Dear ImGui and rlImGui when not installed" ON)

# ============================================================
# Dependencies: use what is installed, otherwise download it
# ============================================================
include(FetchContent)
if (POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW) # extracted files get the extraction time as their timestamp
endif()

# What is downloaded when a package is not installed. Pinned with checksums, so every fresh
# build compiles the same sources.
set(FX2D_YAMLCPP_URL "https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.9.0.tar.gz")
set(FX2D_YAMLCPP_SHA256 "25cb043240f828a8c51beb830569634bc7ac603978e0f69d6b63558dadefd49a")
set(FX2D_RAYLIB_URL "https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz")
set(FX2D_RAYLIB_SHA256 "aea98ecf5bc5c5e0b789a76de0083a21a70457050ea4cc2aec7566935f5e258e")
set(FX2D_IMGUI_URL "https://github.com/ocornut/imgui/archive/refs/tags/v1.92.9b.tar.gz")
set(FX2D_IMGUI_SHA256 "21d8a0a565e85dce943e375db00812c2f3f0ab21f3f0f7964e364a63422d7f99")
set(FX2D_RLIMGUI_URL "https://github.com/raylib-extras/rlImGui/archive/1550009359ad975927f7f0e4a3f47e3f27123ea9.tar.gz")
set(FX2D_RLIMGUI_SHA256 "7d04ff0c996a89190e1050d3c4448ad784479723c76893efea2d731bd5eb839f")

function(fx2d_require_fetch what)
  if (NOT FX2D_FETCH_DEPS)
    message(FATAL_ERROR "Fx2D: ${what} was not found and FX2D_FETCH_DEPS is OFF. Install it, "
                        "or configure with -DFX2D_FETCH_DEPS=ON to download it.")
  endif()
  message(STATUS "Fx2D: ${what} not found; downloading it")
endfunction()

# A downloaded dependency's headers are third-party code: include them as system headers, so the
# project's warning flags, and -Werror, stop at its own code.
function(fx2d_system_includes tgt)
  get_target_property(dirs ${tgt} INTERFACE_INCLUDE_DIRECTORIES)
  if (dirs)
    set_target_properties(${tgt} PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${dirs}")
  endif()
endfunction()

# --- yaml-cpp ---
if (FX2D_YAML)
  find_package(yaml-cpp CONFIG QUIET)
  if (TARGET yaml-cpp::yaml-cpp)
    set(YAMLCPP_TARGET yaml-cpp::yaml-cpp)
  elseif (TARGET yaml-cpp) # MSYS2 exports the target without a namespace
    set(YAMLCPP_TARGET yaml-cpp)
  else()
    fx2d_require_fetch("yaml-cpp")
    set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
    set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(fx2d_yamlcpp URL ${FX2D_YAMLCPP_URL} URL_HASH SHA256=${FX2D_YAMLCPP_SHA256})
    FetchContent_MakeAvailable(fx2d_yamlcpp)
    fx2d_system_includes(yaml-cpp)
    set(YAMLCPP_TARGET yaml-cpp::yaml-cpp)
  endif()
endif()

# --- raylib ---
if (NOT FX2D_HEADLESS)
  find_package(raylib CONFIG QUIET)
  if (TARGET raylib)
    set(RAYLIB_TARGET raylib)
  elseif (TARGET raylib::raylib)
    set(RAYLIB_TARGET raylib::raylib)
  else()
    find_package(PkgConfig QUIET)
    if (PkgConfig_FOUND)
      pkg_check_modules(RAYLIB QUIET raylib)
    endif()
    if (RAYLIB_FOUND)
      add_library(raylib_pkg INTERFACE)
      target_include_directories(raylib_pkg INTERFACE ${RAYLIB_INCLUDE_DIRS})
      target_link_libraries(raylib_pkg INTERFACE ${RAYLIB_LINK_LIBRARIES})
      set(RAYLIB_TARGET raylib_pkg)
    else()
      # raylib is written in C. On Linux it still needs the system's X11 and OpenGL development
      # packages; the installation guide lists them.
      fx2d_require_fetch("raylib")
      enable_language(C)
      set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
      FetchContent_Declare(fx2d_raylib URL ${FX2D_RAYLIB_URL} URL_HASH SHA256=${FX2D_RAYLIB_SHA256})
      FetchContent_MakeAvailable(fx2d_raylib)
      fx2d_system_includes(raylib)
      set(RAYLIB_TARGET raylib)
    endif()
  endif()
endif()

# ============================================================
# ImGui + rlImGui
# Sources placed in lib/imgui and lib/rlImGui are used when present; otherwise they are
# downloaded.
# ============================================================
if (NOT FX2D_HEADLESS)
set(FX2D_IMGUI_DIR   "${CMAKE_CURRENT_SOURCE_DIR}/lib/imgui"   CACHE PATH "Path to Dear ImGui sources")
set(FX2D_RLIMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/rlImGui" CACHE PATH "Path to rlImGui sources")

if (EXISTS "${FX2D_IMGUI_DIR}/imgui.cpp")
  set(FX2D_IMGUI_SOURCE_DIR "${FX2D_IMGUI_DIR}")
else()
  fx2d_require_fetch("Dear ImGui")
  FetchContent_Declare(fx2d_imgui URL ${FX2D_IMGUI_URL} URL_HASH SHA256=${FX2D_IMGUI_SHA256})
  FetchContent_MakeAvailable(fx2d_imgui)
  set(FX2D_IMGUI_SOURCE_DIR "${fx2d_imgui_SOURCE_DIR}")
endif()
if (EXISTS "${FX2D_RLIMGUI_DIR}/rlImGui.cpp")
  set(FX2D_RLIMGUI_SOURCE_DIR "${FX2D_RLIMGUI_DIR}")
else()
  fx2d_require_fetch("rlImGui")
  FetchContent_Declare(fx2d_rlimgui URL ${FX2D_RLIMGUI_URL} URL_HASH SHA256=${FX2D_RLIMGUI_SHA256})
  FetchContent_MakeAvailable(fx2d_rlimgui)
  set(FX2D_RLIMGUI_SOURCE_DIR "${fx2d_rlimgui_SOURCE_DIR}")
endif()

add_library(rlimgui_library STATIC
  "${FX2D_IMGUI_SOURCE_DIR}/imgui.cpp"
  "${FX2D_IMGUI_SOURCE_DIR}/imgui_draw.cpp"
  "${FX2D_IMGUI_SOURCE_DIR}/imgui_tables.cpp"
  "${FX2D_IMGUI_SOURCE_DIR}/imgui_widgets.cpp"
  "${FX2D_RLIMGUI_SOURCE_DIR}/rlImGui.cpp"
)

target_include_directories(rlimgui_library SYSTEM PUBLIC
  "${FX2D_IMGUI_SOURCE_DIR}"
  "${FX2D_RLIMGUI_SOURCE_DIR}"
)

# rlImGui needs raylib headers/types
target_link_libraries(rlimgui_library PUBLIC ${RAYLIB_TARGET})
endif()

# ============================================================
# Fx2D Library
# ============================================================
set(FX2D_CORE_SOURCES
  src/Entity.cpp
  src/Profile.cpp
  src/Scene.cpp
  src/Collisions.cpp
  src/ContactGraph.cpp
  src/Constraints.cpp
  src/Joints.cpp
  src/Query.cpp
)

if (FX2D_YAML)
  list(APPEND FX2D_CORE_SOURCES src/YamlUtils.cpp)
endif()
if (NOT FX2D_HEADLESS)
  list(APPEND FX2D_CORE_SOURCES src/Renderer.cpp)
endif()

add_library(Fx2Dlib STATIC ${FX2D_CORE_SOURCES})
add_library(Fx2D::Fx2D ALIAS Fx2Dlib)

target_include_directories(Fx2Dlib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# The physics depends on nothing outside the C++ standard library.
if (FX2D_YAML)
  target_link_libraries(Fx2Dlib PUBLIC ${YAMLCPP_TARGET})
  target_compile_definitions(Fx2Dlib PUBLIC YAML_CPP_STATIC_DEFINE)
else()
  target_compile_definitions(Fx2Dlib PUBLIC FX2D_NO_YAML)
endif()

if (NOT FX2D_HEADLESS)
  target_link_libraries(Fx2Dlib PUBLIC rlimgui_library ${RAYLIB_TARGET})
endif()

# When TBB's headers are installed, libstdc++ builds std::execution on TBB, and unoptimised builds
# reference it even for the sequential policy the engine uses. Link it whenever it is found; it is
# also what std::execution::par passed to for_each_entity()/transform_entities() runs on.
find_package(TBB QUIET)
if (TARGET TBB::tbb)
  target_link_libraries(Fx2Dlib PUBLIC TBB::tbb)
endif()

# Compile definitions for the library
target_compile_definitions(Fx2Dlib PUBLIC
  NO_FONT_AWESOME
  $<$<CONFIG:Debug>:DEBUG>
)

# ============================================================
# Main executable
# ============================================================
# The viewer and the visual examples load YAML scenes, so they need FX2D_YAML.
if (NOT FX2D_HEADLESS AND FX2D_YAML)
  set(FX2D_BUILD_VIEWER ON)
else()
  set(FX2D_BUILD_VIEWER OFF)
endif()

if (FX2D_BUILD_VIEWER)
  add_executable(${TARGET_NAME} src/main.cpp)
  target_link_libraries(${TARGET_NAME} PRIVATE Fx2Dlib)
endif()

# ============================================================
# Build configuration options
# ============================================================
option(FX2D_WARNINGS   "Enable high warning levels" ON)
option(FX2D_WERROR     "Treat warnings as errors"    OFF)
option(FX2D_NATIVE     "Enable native CPU tuning (SIMD etc.)" OFF)
option(FX2D_IPO        "Enable interprocedural optimization (LTO/IPO) in Release" ON)

# Floating-point contraction (a*b+c fused into one FMA) is a compiler's choice by default:
# GCC and Clang both contract, MSVC does not, and whether a given expression fuses depends on
# the target ISA. That makes a bench A/B between two builds, or between x86 and ARM, compare
# two different sets of arithmetic. Pinning it off costs a little speed and buys results that
# only move when the code moves. It is PUBLIC on Fx2Dlib because the maths lives in headers,
# so a consumer compiling Math.h with contraction on would not match the library it links.
option(FX2D_PIN_FP_CONTRACT "Pin off FP contraction (FMA) so results are build-independent" ON)

# The per-phase step profiler. Off by default: the timers are cheap but not free, and a
# release build should not pay for them. Fx2DBench prints the split when this is on.
option(FX2D_PROFILE "Build with the per-phase step profiler (FxProfile)" OFF)

# FX2D_NATIVE tunes for the machine doing the build, which is right for development and wrong
# for anything you ship, so it is off by default; results are the same bits either way. Set
# FX2D_ARCH_BASELINE to pin a fixed instruction set instead; it takes precedence when non-empty.
# Accepted: sse2, avx2, avx512 (x86) and armv8-a, armv8.2-a (arm64). Empty means "follow
# FX2D_NATIVE".
set(FX2D_ARCH_BASELINE "" CACHE STRING "Fixed ISA baseline for distributable builds")
set_property(CACHE FX2D_ARCH_BASELINE PROPERTY STRINGS "" sse2 avx2 avx512 armv8-a armv8.2-a)

# ============================================================
# Target architecture
# ============================================================
# CMAKE_SYSTEM_PROCESSOR spells the same architecture differently on every platform -- x86_64
# on Linux, AMD64 on Windows, arm64 on macOS, aarch64 on Linux ARM. Normalise it once so the
# tuning flags below can be chosen by architecture rather than by guessing from the compiler.
if (CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(FX2D_ARCH "arm64")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
  set(FX2D_ARCH "x86_64")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "^(i[3-6]86|x86|X86)$")
  set(FX2D_ARCH "x86")
else()
  set(FX2D_ARCH "other")
endif()
message(STATUS "Fx2D target architecture: ${FX2D_ARCH} (CMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR})")

include(CheckCXXCompilerFlag)

# Adds a compile flag only if this compiler accepts it. Every architecture flag goes through
# here: -march=native is rejected by Apple Clang on arm64, /arch: does not exist for MSVC ARM64,
# and -mcpu=native is missing from older Apple Clang -- all of which are hard errors, not
# warnings, so probing is the only way to stay portable.
function(fx2d_add_flag_if_supported tgt flag)
  string(MAKE_C_IDENTIFIER "FX2D_HAS_FLAG_${flag}" probe)
  check_cxx_compiler_flag("${flag}" ${probe})
  if (${probe})
    target_compile_options(${tgt} PRIVATE "${flag}")
  endif()
endfunction()

# ============================================================
# Helper function to apply common compile options
# ============================================================
function(fx2d_tune_target tgt)
  # --- Warnings ---
  if(FX2D_WARNINGS)
    if (MSVC)
      target_compile_options(${tgt} PRIVATE /W4)
      if (FX2D_WERROR) 
        target_compile_options(${tgt} PRIVATE /WX) 
      endif()
    else()
      target_compile_options(${tgt} PRIVATE -Wall -Wextra -Wconversion -Wshadow -Wpedantic)
      if (FX2D_WERROR) 
        target_compile_options(${tgt} PRIVATE -Werror) 
      endif()
    endif()
  endif()

  # --- Optimization per-config (don't overwrite CMake defaults) ---
  if (MSVC)
    target_compile_options(${tgt} PRIVATE $<$<CONFIG:Release>:/O2>)
  else()
    target_compile_options(${tgt} PRIVATE $<$<CONFIG:Release>:-O3>)
  endif()

  # --- Vectoriser budget ---
  # GCC abandons a loop that would need more runtime alias checks than this budget allows
  # (default 10), and the batched contact solve reads and writes around twenty columns. Without
  # this the two hot solve loops silently fall back to scalar; with it they vectorise to 32-byte
  # vectors, measured at 26% off the `stacks` step and 14% off `pile`.
  #
  # Gated on the compiler being GCC rather than probed: Clang *accepts* the spelling and then
  # warns that it went unused, which -Werror turns into an error. Clang has its own budget and
  # vectorises these loops without help.
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${tgt} PRIVATE "--param=vect-max-version-for-alias-checks=200")
  endif()

  # --- CPU features (auto-vectorization benefits) ---
  if (EMSCRIPTEN)
    # WebAssembly has no host CPU to tune for, so the browser build takes no ISA flags.
  elseif (NOT FX2D_ARCH_BASELINE STREQUAL "")
    # A fixed baseline: the binary may assume this ISA wherever it runs.
    if (MSVC)
      if (FX2D_ARCH_BASELINE STREQUAL "sse2" AND FX2D_ARCH STREQUAL "x86")
        target_compile_options(${tgt} PRIVATE /arch:SSE2)
      elseif (FX2D_ARCH_BASELINE STREQUAL "avx2")
        target_compile_options(${tgt} PRIVATE /arch:AVX2)
      elseif (FX2D_ARCH_BASELINE STREQUAL "avx512")
        target_compile_options(${tgt} PRIVATE /arch:AVX512)
      endif()
      # x86-64 MSVC is SSE2 by default and ARM64 MSVC is NEON by default, so those two
      # baselines need no flag at all.
    elseif (FX2D_ARCH_BASELINE STREQUAL "sse2")
      fx2d_add_flag_if_supported(${tgt} "-msse2")
    elseif (FX2D_ARCH_BASELINE STREQUAL "avx2")
      fx2d_add_flag_if_supported(${tgt} "-mavx2")
      fx2d_add_flag_if_supported(${tgt} "-mfma")
    elseif (FX2D_ARCH_BASELINE STREQUAL "avx512")
      fx2d_add_flag_if_supported(${tgt} "-mavx512f")
    elseif (FX2D_ARCH_BASELINE MATCHES "^armv8")
      fx2d_add_flag_if_supported(${tgt} "-march=${FX2D_ARCH_BASELINE}")
    else()
      message(WARNING "Fx2D: unknown FX2D_ARCH_BASELINE '${FX2D_ARCH_BASELINE}', ignoring")
    endif()
  elseif (FX2D_NATIVE)
    # Tune for the machine doing the build. Never use this for a binary you ship.
    if (MSVC)
      if (FX2D_ARCH STREQUAL "x86_64" OR FX2D_ARCH STREQUAL "x86")
        # AVX2 is a solid baseline; bump to AVX512 if you know your CPUs
        target_compile_options(${tgt} PRIVATE /arch:AVX2)
      endif()
      # MSVC ARM64 rejects /arch: entirely. NEON is mandatory in ARMv8, so there is nothing
      # to ask for -- the vector unit is already assumed.
    elseif (FX2D_ARCH STREQUAL "arm64")
      # -march=native is an error on Apple Clang for arm64; -mcpu=native is the spelling that
      # works on both GCC and Clang, and the probe covers Apple Clang versions predating it.
      fx2d_add_flag_if_supported(${tgt} "-mcpu=native")
    else()
      fx2d_add_flag_if_supported(${tgt} "-march=native")
      fx2d_add_flag_if_supported(${tgt} "-mtune=native")
    endif()
  endif()

  # --- Determinism: no silent FMA contraction (see FX2D_PIN_FP_CONTRACT) ---
  if (FX2D_PIN_FP_CONTRACT)
    if (MSVC)
      target_compile_options(${tgt} PRIVATE /fp:precise)
    else()
      target_compile_options(${tgt} PRIVATE -ffp-contract=off)
    endif()
  endif()

  # --- Sanitizers in Debug (helpful; skip on MSVC by default) ---
  if (NOT MSVC)
    target_compile_options(${tgt} PRIVATE $<$<CONFIG:Debug>:-fsanitize=address,undefined>)
    target_link_options(${tgt}    PRIVATE $<$<CONFIG:Debug>:-fsanitize=address,undefined>)
  endif()
endfunction()

# ============================================================
# Apply tuning to targets
# ============================================================
fx2d_tune_target(Fx2Dlib)
if (FX2D_BUILD_VIEWER)
  fx2d_tune_target(${TARGET_NAME})
endif()

# Both of these have to reach anything that includes the headers, not just the library's own
# translation units: FxProfile's storage is only defined when the library was built with the
# profiler on, and the FP-contract pin is meaningless if a consumer compiles Math.h differently.
if (FX2D_PROFILE)
  target_compile_definitions(Fx2Dlib PUBLIC FX2D_PROFILE)
endif()
if (FX2D_PIN_FP_CONTRACT)
  if (MSVC)
    target_compile_options(Fx2Dlib PUBLIC /fp:precise)
  else()
    target_compile_options(Fx2Dlib PUBLIC -ffp-contract=off)
  endif()
endif()

# ============================================================
# Examples
# ============================================================
# Each example resolves its Scene.yml relative to the working directory, so run
# the resulting binaries from the repo root.
# Headless builds get the two renderer-free examples; full builds get all of them.
option(FX2D_BUILD_EXAMPLES "Build the example programs" OFF)

if (FX2D_BUILD_EXAMPLES AND NOT FX2D_YAML)
  message(STATUS "Fx2D: the examples load YAML scenes; skipping them because FX2D_YAML is OFF")
endif()

if (FX2D_BUILD_EXAMPLES AND FX2D_YAML)
  add_executable(truck_headless         examples/truck/main_headless.cpp)
  add_executable(joint_control_headless examples/joint_control_demo/main.cpp)
  foreach(example_target truck_headless joint_control_headless)
    target_link_libraries(${example_target} PRIVATE Fx2Dlib)
    fx2d_tune_target(${example_target})
  endforeach()
endif()

if (FX2D_BUILD_EXAMPLES AND FX2D_BUILD_VIEWER)
  add_executable(example_truck         examples/truck/main.cpp)
  add_executable(example_joint_control examples/joint_control_demo/main_visual.cpp)
  add_executable(example_stacked_boxes examples/stacked_boxes/main.cpp)
  add_executable(example_angry_boxes   examples/angry_boxes/main.cpp)
  add_executable(example_chain_terrain examples/chain_terrain/main.cpp)
  add_executable(example_bucket_fill   examples/bucket_fill/main.cpp)
  add_executable(example_playground    examples/playground/main.cpp)

  foreach(example_target example_truck example_joint_control example_stacked_boxes example_angry_boxes
                         example_chain_terrain example_bucket_fill example_playground)
    target_link_libraries(${example_target} PRIVATE Fx2Dlib)
    fx2d_tune_target(${example_target})
  endforeach()
endif()

# ============================================================
# Browser playground (Emscripten)
# ============================================================
# Configure through emcmake; scripts/build_web.sh does the whole thing, dependencies included.
option(FX2D_BUILD_WEB "Build the browser playground (needs the Emscripten toolchain)" OFF)

if (FX2D_BUILD_WEB)
  if (NOT EMSCRIPTEN)
    message(FATAL_ERROR "FX2D_BUILD_WEB needs the Emscripten toolchain: configure with emcmake.")
  endif()
  add_executable(playground_web examples/playground/main.cpp)
  target_link_libraries(playground_web PRIVATE Fx2Dlib)
  fx2d_tune_target(playground_web)
  set_target_properties(playground_web PROPERTIES OUTPUT_NAME playground SUFFIX ".js")
  # The scene file is read at the same relative path the desktop build uses.
  target_link_options(playground_web PRIVATE
    -sUSE_GLFW=3 -sALLOW_MEMORY_GROWTH=1 -sASSERTIONS=0 -sSTACK_SIZE=1MB
    "--preload-file=${CMAKE_CURRENT_SOURCE_DIR}/examples/playground/Scene.yml@examples/playground/Scene.yml")
endif()

# ============================================================
# Benchmark
# ============================================================
# Renderer-free, so it builds headless. Threading changes need an A/B measurement before they
# land (see docs/engineering-notes.md); this is what produces it.
option(FX2D_BUILD_BENCH "Build the step-time benchmark" OFF)

if (FX2D_BUILD_BENCH)
  add_executable(Fx2DBench scripts/bench.cpp)
  target_link_libraries(Fx2DBench PRIVATE Fx2Dlib)
  fx2d_tune_target(Fx2DBench)
endif()

if (FX2D_BUILD_TESTS)
  enable_testing()

  set(FX2D_TEST_SOURCES
    tests/main.cpp
    tests/test_aabb_tree.cpp
    tests/test_ccd.cpp
    tests/test_angle_precision.cpp
    tests/test_resting_stability.cpp
    tests/test_contact_events.cpp
    tests/test_adversarial.cpp
    tests/test_input.cpp
    tests/test_queries.cpp
    tests/test_groups.cpp
    tests/test_slingshot.cpp
    tests/test_contact_graph.cpp
    tests/test_solver_regression.cpp
  )
  # These suites load YAML scenes. Without FX2D_YAML they are left out, and tests/main.cpp
  # skips them, so the dependency-free build still runs the rest.
  if (FX2D_YAML)
    list(APPEND FX2D_TEST_SOURCES
      tests/test_joints.cpp
      tests/test_capsule_collision.cpp
      tests/test_collisions_edge.cpp
      tests/test_yaml_inertia.cpp
      tests/test_chain.cpp
      tests/test_mouse_joint.cpp
    )
  endif()

  add_executable(Fx2DTests ${FX2D_TEST_SOURCES})

  target_link_libraries(Fx2DTests PRIVATE Fx2Dlib)
  # test_slingshot.cpp drives the example's FxSlingshot directly, so the mechanic that ships
  # in examples/angry_boxes is the one under test.
  target_include_directories(Fx2DTests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples)
  fx2d_tune_target(Fx2DTests)
  add_test(NAME Fx2DTests COMMAND Fx2DTests)
endif()

# ============================================================
# Interprocedural Optimization (LTO/IPO)
# ============================================================
if (FX2D_IPO)
  cmake_policy(SET CMP0069 NEW)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_ok OUTPUT ipo_msg)
  if (ipo_ok)
    set_property(TARGET Fx2Dlib       PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    if (TARGET ${TARGET_NAME})
      set_property(TARGET ${TARGET_NAME} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    endif()
  else()
    message(STATUS "IPO not supported: ${ipo_msg}")
  endif()
endif()

# ============================================================
# Build status messages
# ============================================================
message(STATUS "Raylib target: ${RAYLIB_TARGET}")
message(STATUS "yaml-cpp target: ${YAMLCPP_TARGET}")
message(STATUS "ImGui source dir: ${FX2D_IMGUI_SOURCE_DIR}")
message(STATUS "rlImGui source dir: ${FX2D_RLIMGUI_SOURCE_DIR}")
if (FX2D_BUILD_VIEWER)
  message(STATUS "Executable will be: ${CMAKE_BINARY_DIR}/${TARGET_NAME}")
endif()
