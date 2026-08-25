---
title: Link Fx2Dlib
---

# Link `Fx2Dlib`

Fx2D’s CMake project creates one static-library target: `Fx2Dlib`. Link your executable to that target; its public include path and required Eigen/yaml-cpp dependencies propagate through the target.

Fx2D does **not** currently export an installed package configuration, so this will not work yet:

```cmake
find_package(Fx2D CONFIG REQUIRED) # not available
```

Instead, add the source tree to your CMake build.

## Add a checked-out copy

```cmake
# Your CMakeLists.txt
set(FX2D_HEADLESS ON CACHE BOOL "Build Fx2D without raylib/ImGui" FORCE)
set(FX2D_BUILD_TESTS OFF CACHE BOOL "Skip Fx2D's test executable" FORCE)
add_subdirectory(external/fx2d)

add_executable(my_sim main.cpp)
target_link_libraries(my_sim PRIVATE Fx2Dlib)
```

Use `FX2D_HEADLESS=ON` for simulations, CI, and tooling. It excludes `Renderer.cpp`, the viewer executable, and visual examples; raylib and ImGui are not needed.

## Fetch with CMake

```cmake
include(FetchContent)

set(FX2D_HEADLESS ON CACHE BOOL "" FORCE)
set(FX2D_BUILD_TESTS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  fx2d
  GIT_REPOSITORY https://github.com/Bharath2/fx2d-physics-engine.git
  GIT_TAG main # Prefer a pinned commit in reproducible projects.
)
FetchContent_MakeAvailable(fx2d)

add_executable(my_sim main.cpp)
target_link_libraries(my_sim PRIVATE Fx2Dlib)
```

## Enable the renderer

For a visual application, leave `FX2D_HEADLESS` off:

```cmake
set(FX2D_HEADLESS OFF CACHE BOOL "" FORCE)
set(FX2D_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(external/fx2d)

add_executable(my_viewer main.cpp)
target_link_libraries(my_viewer PRIVATE Fx2Dlib)
```

Before configuring, populate Fx2D’s placeholder `lib/imgui/` and `lib/rlImGui/` directories with their upstream source files. The renderer build also needs raylib, yaml-cpp, and Eigen3 discoverable by CMake. The top-level [installation guide](/getting-started/install) has the full dependency list.

## Build options

| Option | Default | Effect |
|---|---:|---|
| `FX2D_HEADLESS` | `OFF` | Remove raylib/ImGui renderer code and visual examples. |
| `FX2D_BUILD_TESTS` | `ON` | Build the `Fx2DTests` CTest executable. |
| `FX2D_BUILD_EXAMPLES` | `OFF` | Build visual sample targets. Mutually exclusive with headless mode. |
| `FX2D_BUILD_BENCH` | `OFF` | Build the `Fx2DBench` headless benchmark. |
| `FX2D_WARNINGS` | `ON` | Enable the project’s higher compiler warning level. |
| `FX2D_WERROR` | `OFF` | Treat warnings as errors. |

See [header selection](/api/headers) for what your application should include once it links `Fx2Dlib`.
