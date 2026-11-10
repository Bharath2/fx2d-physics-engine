---
title: Installation
---

# Installation

Fx2D is a C++20 project built with CMake. The physics library needs nothing beyond a C++20 compiler. The YAML scene loader uses yaml-cpp, and the visual viewer uses raylib, Dear ImGui, and rlImGui. The engine also supports a headless build for CI and non-rendered simulations.

## Clone and prepare dependencies

```bash
git clone https://github.com/Bharath2/fx2d-physics-engine.git
cd fx2d-physics-engine
```

Install CMake 3.16+. Whatever CMake cannot find among yaml-cpp, raylib, Dear ImGui, and rlImGui is downloaded at a pinned version and built with the project, so a fresh clone configures without installing packages first. Installed packages are used when present, and so are sources placed in `lib/imgui` and `lib/rlImGui`. To require installed packages instead, configure with `-DFX2D_FETCH_DEPS=OFF`.

On Linux, raylib still needs the system's window and OpenGL development packages. On Debian/Ubuntu, raylib's Linux guide lists:

```bash
sudo apt install libasound2-dev libx11-dev libxrandr-dev libxi-dev libgl1-mesa-dev \
  libglu1-mesa-dev libxcursor-dev libxinerama-dev libwayland-dev libxkbcommon-dev
```

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
cmake -S . -B build-headless -DCMAKE_BUILD_TYPE=Release -DFX2D_HEADLESS=ON -DFX2D_BUILD_EXAMPLES=ON
cmake --build build-headless -j
./build-headless/Fx2DTests
./build-headless/truck_headless
```

Add `-DFX2D_YAML=OFF` and the library builds from the C++ standard library alone; scenes are then built in code rather than loaded from YAML.

Continue with [your first scene](/getting-started/first-scene), or read the full [headless guide](/guides/headless).
