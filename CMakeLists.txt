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

# Helpful on MSYS2/MinGW users (optional): prefer common prefixes
list(APPEND CMAKE_PREFIX_PATH
  "C:/msys64/mingw64"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib"
)

# ============================================================
# Dependencies (prefer imported targets; fall back to pkg-config)
# ============================================================
# --- raylib ---
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

# ============================================================
# Fx2D Library
# ============================================================
add_library(Fx2Dlib STATIC
  src/Entity.cpp
  src/Scene.cpp
  src/Collisions.cpp
  src/Constraints.cpp
  src/Joints.cpp
  src/Renderer.cpp
  src/YamlUtils.cpp
)

target_include_directories(Fx2Dlib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to the library
target_link_libraries(Fx2Dlib PUBLIC
  rlimgui_library
  Eigen3::Eigen
  ${YAMLCPP_TARGET}
  ${RAYLIB_TARGET}
)

# Compile definitions for the library
target_compile_definitions(Fx2Dlib PUBLIC
  NO_FONT_AWESOME
  YAML_CPP_STATIC_DEFINE
  $<$<CONFIG:Debug>:DEBUG>
)

# ============================================================
# Main executable
# ============================================================
add_executable(${TARGET_NAME}
  src/main.cpp
)

# Link the library to the executable
target_link_libraries(${TARGET_NAME} PRIVATE Fx2Dlib)

# ============================================================
# Build configuration options
# ============================================================
option(FX2D_WARNINGS   "Enable high warning levels" ON)
option(FX2D_WERROR     "Treat warnings as errors"    OFF)
option(FX2D_NATIVE     "Enable native CPU tuning (SIMD etc.)" ON)
option(FX2D_IPO        "Enable interprocedural optimization (LTO/IPO) in Release" ON)

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

  # --- CPU features (auto-vectorization benefits) ---
  if (FX2D_NATIVE)
    if (MSVC)
      # AVX2 is a solid baseline; bump to AVX512 if you know your CPUs
      target_compile_options(${tgt} PRIVATE /arch:AVX2)
    else()
      target_compile_options(${tgt} PRIVATE -march=native -mtune=native)
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
fx2d_tune_target(${TARGET_NAME})

if (FX2D_BUILD_TESTS)
  enable_testing()

  add_executable(Fx2DTests
    tests/test_aabb_tree.cpp
    tests/test_joints.cpp
    tests/test_ccd.cpp
    tests/test_capsule_collision.cpp
    tests/test_collisions_edge.cpp
    tests/test_angle_precision.cpp
  )

  target_link_libraries(Fx2DTests PRIVATE Fx2Dlib)
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
