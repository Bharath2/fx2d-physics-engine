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
# Local third-party sources: imgui + rlImGui
# Built as a static lib that the app links against.
# ============================================================
add_library(rlimgui_library STATIC
  lib/imgui/imgui.cpp
  lib/imgui/imgui_draw.cpp
  lib/imgui/imgui_tables.cpp
  lib/imgui/imgui_widgets.cpp
  lib/rlImGui/rlImGui.cpp
)

target_include_directories(rlimgui_library PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/imgui"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/rlImGui"
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

# Compile defs for the library
target_compile_definitions(Fx2Dlib PUBLIC
  NO_FONT_AWESOME
  YAML_CPP_STATIC_DEFINE
  $<$<CONFIG:Debug>:DEBUG>
)

# Warnings for the library
if (FX2D_WARNINGS)
  if (MSVC)
    target_compile_options(Fx2Dlib PRIVATE /W4)
    if (FX2D_WERROR)
      target_compile_options(Fx2Dlib PRIVATE /WX)
    endif()
  else()
    target_compile_options(Fx2Dlib PRIVATE -Wall -Wextra)
    if (FX2D_WERROR)
      target_compile_options(Fx2Dlib PRIVATE -Werror)
    endif()
  endif()
endif()

# ============================================================
# Main executable
# ============================================================
add_executable(${TARGET_NAME}
  src/main.cpp
)

# Link the library to the executable
target_link_libraries(${TARGET_NAME} PRIVATE
  Fx2Dlib
)

# Optimizations / Debug
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG   "-g")

# Nice status
message(STATUS "Raylib target: ${RAYLIB_TARGET}")
message(STATUS "yaml-cpp target: ${YAMLCPP_TARGET}")
message(STATUS "Executable will be: ${CMAKE_BINARY_DIR}/${TARGET_NAME}")
