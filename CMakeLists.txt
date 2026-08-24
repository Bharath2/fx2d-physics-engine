# ============================================================
# Project Definition
# ============================================================
cmake_minimum_required(VERSION 3.16)
project(Fx2DProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(TARGET_NAME Fx2D)

# Options
option(FX2D_WARNINGS "Enable extra warnings" ON)
option(FX2D_WERROR   "Treat warnings as errors" OFF)
option(FX2D_BUILD_TESTS "Build unit tests" ON)

# Headless builds the physics core and tests without raylib/ImGui, which are not
# vendored in the repo. Renderer.cpp, the viewer executable, and the visual
# examples are excluded. This is what CI builds.
option(FX2D_HEADLESS "Build without the renderer (no raylib/ImGui)" OFF)

# Helpful on MSYS2/MinGW users (optional): prefer common prefixes
list(APPEND CMAKE_PREFIX_PATH
  "C:/msys64/mingw64"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib"
)

# ============================================================
# Dependencies (prefer imported targets; fall back to pkg-config)
# ============================================================
# --- raylib ---
if (NOT FX2D_HEADLESS)
find_package(raylib CONFIG QUIET)
if (raylib_FOUND)
  if (TARGET raylib)
    set(RAYLIB_TARGET raylib)
  elseif (TARGET raylib::raylib)
    set(RAYLIB_TARGET raylib::raylib)
  else()
    message(FATAL_ERROR "raylib was found but exposes no usable target.")
  endif()
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(RAYLIB REQUIRED raylib)
  add_library(raylib_pkg INTERFACE)
  target_include_directories(raylib_pkg INTERFACE ${RAYLIB_INCLUDE_DIRS})
  target_link_libraries(raylib_pkg INTERFACE ${RAYLIB_LINK_LIBRARIES})
  set(RAYLIB_TARGET raylib_pkg)
endif()
endif()

# --- yaml-cpp ---
# MSYS2 exports yaml-cpp CONFIG target named 'yaml-cpp'
find_package(yaml-cpp CONFIG QUIET)
if (TARGET yaml-cpp)
  set(YAMLCPP_TARGET yaml-cpp)
else()
  # Fallback to module mode or pkg-config
  find_package(yaml-cpp QUIET)
  if (TARGET yaml-cpp::yaml-cpp)
    set(YAMLCPP_TARGET yaml-cpp::yaml-cpp)
  else()
    pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
    add_library(yamlcpp_pkg INTERFACE)
    target_include_directories(yamlcpp_pkg INTERFACE ${YAMLCPP_INCLUDE_DIRS})
    target_link_libraries(yamlcpp_pkg INTERFACE ${YAMLCPP_LINK_LIBRARIES})
    set(YAMLCPP_TARGET yamlcpp_pkg)
  endif()
endif()

# --- Eigen3 (header-only) ---
find_package(Eigen3 3.3 REQUIRED)

# ============================================================
# ImGui + rlImGui
# These folders are kept as placeholders in Git and should be populated locally.
# ============================================================
if (NOT FX2D_HEADLESS)
set(FX2D_IMGUI_DIR   "${CMAKE_CURRENT_SOURCE_DIR}/lib/imgui"   CACHE PATH "Path to Dear ImGui sources")
set(FX2D_RLIMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/rlImGui" CACHE PATH "Path to rlImGui sources")

if (NOT EXISTS "${FX2D_IMGUI_DIR}/imgui.cpp")
  message(FATAL_ERROR "Dear ImGui sources were not found. Populate lib/imgui/ locally before building.")
endif()
if (NOT EXISTS "${FX2D_RLIMGUI_DIR}/rlImGui.cpp")
  message(FATAL_ERROR "rlImGui sources were not found. Populate lib/rlImGui/ locally before building.")
endif()

set(FX2D_IMGUI_SOURCE_DIR "${FX2D_IMGUI_DIR}")
set(FX2D_RLIMGUI_SOURCE_DIR "${FX2D_RLIMGUI_DIR}")

add_library(rlimgui_library STATIC
  "${FX2D_IMGUI_SOURCE_DIR}/imgui.cpp"
  "${FX2D_IMGUI_SOURCE_DIR}/imgui_draw.cpp"
  "${FX2D_IMGUI_SOURCE_DIR}/imgui_tables.cpp"
  "${FX2D_IMGUI_SOURCE_DIR}/imgui_widgets.cpp"
  "${FX2D_RLIMGUI_SOURCE_DIR}/rlImGui.cpp"
)

target_include_directories(rlimgui_library PUBLIC
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
  src/YamlUtils.cpp
)

if (NOT FX2D_HEADLESS)
  list(APPEND FX2D_CORE_SOURCES src/Renderer.cpp)
endif()

add_library(Fx2Dlib STATIC ${FX2D_CORE_SOURCES})

target_include_directories(Fx2Dlib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to the library
target_link_libraries(Fx2Dlib PUBLIC
  Eigen3::Eigen
  ${YAMLCPP_TARGET}
)

if (NOT FX2D_HEADLESS)
  target_link_libraries(Fx2Dlib PUBLIC rlimgui_library ${RAYLIB_TARGET})
endif()

# libstdc++ implements the parallel std::execution policies on top of TBB. The engine itself
# runs sequentially (see the note above FxScene::step), but for_each_entity/transform_entities
# take a policy from the caller, so TBB stays linked to keep std::execution::par usable.
find_package(TBB QUIET)
if (TARGET TBB::tbb)
  target_link_libraries(Fx2Dlib PUBLIC TBB::tbb)
endif()

# Compile definitions for the library
target_compile_definitions(Fx2Dlib PUBLIC
  NO_FONT_AWESOME
  YAML_CPP_STATIC_DEFINE
  $<$<CONFIG:Debug>:DEBUG>
)

# ============================================================
# Main executable
# ============================================================
if (NOT FX2D_HEADLESS)
  add_executable(${TARGET_NAME} src/main.cpp)
  target_link_libraries(${TARGET_NAME} PRIVATE Fx2Dlib)
endif()

# ============================================================
# Build configuration options
# ============================================================
option(FX2D_WARNINGS   "Enable high warning levels" ON)
option(FX2D_WERROR     "Treat warnings as errors"    OFF)
option(FX2D_NATIVE     "Enable native CPU tuning (SIMD etc.)" ON)
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
# for anything you ship. Set FX2D_ARCH_BASELINE instead to pin a fixed instruction set that the
# binary can assume everywhere it runs; it takes precedence over FX2D_NATIVE when non-empty.
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
  if (NOT FX2D_ARCH_BASELINE STREQUAL "")
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
if (NOT FX2D_HEADLESS)
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
option(FX2D_BUILD_EXAMPLES "Build the example programs" OFF)

if (FX2D_BUILD_EXAMPLES AND FX2D_HEADLESS)
  message(FATAL_ERROR "The example programs need the renderer; FX2D_BUILD_EXAMPLES and FX2D_HEADLESS are mutually exclusive.")
endif()

if (FX2D_BUILD_EXAMPLES)
  add_executable(example_truck         examples/truck/main.cpp)
  add_executable(example_joint_control examples/joint_control_demo/main_visual.cpp)
  add_executable(example_stacked_boxes examples/stacked_boxes/main.cpp)
  add_executable(example_angry_boxes   examples/angry_boxes/main.cpp)
  add_executable(example_chain_terrain examples/chain_terrain/main.cpp)
  add_executable(example_bucket_fill   examples/bucket_fill/main.cpp)

  foreach(example_target example_truck example_joint_control example_stacked_boxes example_angry_boxes
                         example_chain_terrain example_bucket_fill)
    target_link_libraries(${example_target} PRIVATE Fx2Dlib)
    fx2d_tune_target(${example_target})
  endforeach()
endif()

# ============================================================
# Benchmark
# ============================================================
# Renderer-free, so it builds headless. ToDo item 7 requires an A/B measurement before any
# threading change lands; this is what produces it.
option(FX2D_BUILD_BENCH "Build the step-time benchmark" OFF)

if (FX2D_BUILD_BENCH)
  add_executable(Fx2DBench scripts/bench.cpp)
  target_link_libraries(Fx2DBench PRIVATE Fx2Dlib)
  fx2d_tune_target(Fx2DBench)
endif()

if (FX2D_BUILD_TESTS)
  enable_testing()

  add_executable(Fx2DTests
    tests/main.cpp
    tests/test_aabb_tree.cpp
    tests/test_joints.cpp
    tests/test_ccd.cpp
    tests/test_capsule_collision.cpp
    tests/test_collisions_edge.cpp
    tests/test_angle_precision.cpp
    tests/test_resting_stability.cpp
    tests/test_contact_events.cpp
    tests/test_yaml_inertia.cpp
    tests/test_adversarial.cpp
    tests/test_input.cpp
    tests/test_queries.cpp
    tests/test_chain.cpp
    tests/test_groups.cpp
    tests/test_slingshot.cpp
    tests/test_contact_graph.cpp
    tests/test_solver_regression.cpp
  )

  target_link_libraries(Fx2DTests PRIVATE Fx2Dlib)
  # test_slingshot.cpp drives the example's FxSlingshot directly, so the mechanic that ships
  # in examples/angry_boxes is the one under test.
  target_include_directories(Fx2DTests PRIVATE ${CMAKE_SOURCE_DIR}/examples)
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
    set_property(TARGET ${TARGET_NAME} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
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
message(STATUS "Executable will be: ${CMAKE_BINARY_DIR}/${TARGET_NAME}")
