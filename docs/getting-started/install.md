---
title: Installation
---

# Installation

Fx2D is a C++20 project built with CMake. The visual viewer uses raylib, Dear ImGui, rlImGui, yaml-cpp, and Eigen3. The engine also supports a headless build for CI and non-rendered simulations.

## Clone and prepare dependencies

```bash
git clone https://github.com/Bharath2/fx2d-physics-engine.git
cd fx2d-physics-engine
```

Install CMake 3.16+, Eigen3 3.3+, yaml-cpp, and raylib 4.5+. For visual builds, populate the placeholder `lib/imgui` and `lib/rlImGui` folders with their upstream sources before configuring CMake.

## Build the viewer

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/Fx2D
```

The `fxmake` helper offers equivalent shortcuts:

```bash
./fxmake          # release build
./fxmake debug    # debug build
```

## Build headless

For tests, batch simulations, or code that does not need the graphics stack:

```bash
./scripts/build_headless.sh
cmake --build build-headless -j
```

Continue with [your first scene](/getting-started/first-scene), or read the full [headless guide](/guides/headless).
