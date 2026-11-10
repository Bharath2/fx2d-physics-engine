# ============================================================
# Fx2D against Box2D v3, on the scenes in scripts/bench_scenes.h
# ============================================================
# A separate project so the main build never downloads Box2D. From the repository root:
#   cmake -S benchmarks/box2d -B build-box2d -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-box2d -j && ./build-box2d/fx2d_vs_box2d
cmake_minimum_required(VERSION 3.22) # Box2D's own minimum
project(Fx2DVsBox2D LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fx2D as a subproject: the headless physics library only, which needs no packages.
set(FX2D_YAML OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../.. fx2d)

include(FetchContent)
if (POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()
set(BOX2D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX2D_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BOX2D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX2D_DOCS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(box2d
  URL https://github.com/erincatto/box2d/archive/refs/tags/v3.2.0.tar.gz
  URL_HASH SHA256=ffec747b27a2267016f8d0240d658c82d187fb2271fa0f69fcbe53fbc067180d
)
FetchContent_MakeAvailable(box2d)

add_executable(fx2d_vs_box2d compare.cpp)
target_include_directories(fx2d_vs_box2d PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts)
target_link_libraries(fx2d_vs_box2d PRIVATE Fx2D::Fx2D box2d::box2d)
