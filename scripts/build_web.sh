#!/usr/bin/env bash
# Builds the browser playground (examples/playground) with Emscripten and drops the result in
# docs/public/playground/, where the documentation site serves it.
#
# Needs an activated emsdk (emcc on PATH), git and cmake. Everything third-party is fetched
# and built once under build-web/: raylib for the web target, yaml-cpp, Eigen headers, and
# Dear ImGui + rlImGui into lib/ if those placeholder folders are still empty.
#
# Usage: ./scripts/build_web.sh
set -euo pipefail
cd "$(dirname "$0")/.."

OUT=build-web
PREFIX="$PWD/$OUT/prefix"
mkdir -p "$OUT" "$PREFIX"

command -v emcc >/dev/null || { echo "error: emcc not on PATH; source emsdk_env.sh first" >&2; exit 1; }

clone() { # clone <url> <dir> [ref]
  [[ -d "$2" ]] && return 0
  git clone -q --depth 1 ${3:+--branch "$3"} "$1" "$2"
}

# --- renderer sources the repo does not vendor
[[ -f lib/imgui/imgui.cpp ]]     || { rm -rf lib/imgui;   clone https://github.com/ocornut/imgui.git lib/imgui v1.92.9b; }
[[ -f lib/rlImGui/rlImGui.cpp ]] || { rm -rf lib/rlImGui; clone https://github.com/raylib-extras/rlImGui.git lib/rlImGui; }

# --- raylib for the web
if [[ ! -f "$PREFIX/lib/libraylib.a" ]]; then
  clone https://github.com/raysan5/raylib.git "$OUT/raylib" 5.5
  emcmake cmake -S "$OUT/raylib" -B "$OUT/raylib-build" -DPLATFORM=Web -DBUILD_EXAMPLES=OFF \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" >/dev/null
  cmake --build "$OUT/raylib-build" -j
  cmake --install "$OUT/raylib-build" >/dev/null
fi

# --- yaml-cpp
if [[ ! -f "$PREFIX/lib/libyaml-cpp.a" ]]; then
  clone https://github.com/jbeder/yaml-cpp.git "$OUT/yaml-cpp" 0.8.0
  emcmake cmake -S "$OUT/yaml-cpp" -B "$OUT/yaml-cpp-build" -DYAML_CPP_BUILD_TESTS=OFF \
    -DYAML_CPP_BUILD_TOOLS=OFF -DYAML_BUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" >/dev/null
  cmake --build "$OUT/yaml-cpp-build" -j
  cmake --install "$OUT/yaml-cpp-build" >/dev/null
fi

# --- Eigen (headers only; its CMake install needs no build)
if [[ ! -d "$PREFIX/include/eigen3" ]]; then
  clone https://gitlab.com/libeigen/eigen.git "$OUT/eigen" 3.4.0
  cmake -S "$OUT/eigen" -B "$OUT/eigen-build" -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DEIGEN_BUILD_DOC=OFF -DBUILD_TESTING=OFF >/dev/null
  cmake --install "$OUT/eigen-build" >/dev/null
fi

# --- the engine and the playground
emcmake cmake -S . -B "$OUT/fx2d" -DCMAKE_BUILD_TYPE=Release -DFX2D_BUILD_WEB=ON \
  -DFX2D_BUILD_TESTS=OFF -DCMAKE_PREFIX_PATH="$PREFIX" -DCMAKE_FIND_ROOT_PATH="$PREFIX"
cmake --build "$OUT/fx2d" -j --target playground_web

mkdir -p docs/public/playground
cp "$OUT/fx2d/playground.js" "$OUT/fx2d/playground.wasm" "$OUT/fx2d/playground.data" docs/public/playground/
echo
echo "Playground written to docs/public/playground/ — open docs/public/playground/index.html through the docs dev server."
