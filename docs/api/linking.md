---
title: Link Fx2Dlib
---

# Link `Fx2Dlib`

Fx2D’s CMake project creates one static-library target: `Fx2Dlib`, also available as `Fx2D::Fx2D`. Link your executable to that target; its public include path and its yaml-cpp dependency propagate through the target.

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

Use `FX2D_HEADLESS=ON` for simulations, CI, and tooling. It excludes `Renderer.cpp`, the viewer executable, and visual examples; raylib and ImGui are not needed. Added this way, Fx2D already defaults to headless with no tests, so the two `set()` lines only make that explicit. Set `FX2D_YAML` off as well and `Fx2Dlib` needs nothing beyond the C++ standard library.

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

Any of raylib, yaml-cpp, Dear ImGui, and rlImGui that CMake cannot find is downloaded and built with your project. The top-level [installation guide](/getting-started/install) covers the system packages raylib needs on Linux.

## Build options

| Option | Default | Effect |
|---|---:|---|
| `FX2D_HEADLESS` | `OFF` (`ON` as a subproject) | Remove raylib/ImGui renderer code and visual examples. |
| `FX2D_YAML` | `ON` | Build the YAML scene loader (`FxYAML`), the only part that uses yaml-cpp. |
| `FX2D_FETCH_DEPS` | `ON` | Download yaml-cpp, raylib, Dear ImGui, and rlImGui when they are not installed. |
| `FX2D_BUILD_TESTS` | `ON` (`OFF` as a subproject) | Build the `Fx2DTests` CTest executable. |
| `FX2D_BUILD_EXAMPLES` | `OFF` | Build the sample targets; headless builds get only the two renderer-free ones. |
| `FX2D_BUILD_BENCH` | `OFF` | Build the `Fx2DBench` headless benchmark. |
| `FX2D_WARNINGS` | `ON` | Enable the project’s higher compiler warning level. |
| `FX2D_WERROR` | `OFF` | Treat warnings as errors. |

See [header selection](/api/headers) for what your application should include once it links `Fx2Dlib`.
