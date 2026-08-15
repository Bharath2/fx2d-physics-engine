#!/usr/bin/env bash
# Headless g++ build (no raylib/ImGui). Needs: eigen3, yaml-cpp, tbb.
# Usage: ./scripts/build_headless.sh && ./build-headless/truck_headless
set -euo pipefail

cd "$(dirname "$0")/.."
OUT=build-headless
mkdir -p "$OUT"

CXX=${CXX:-g++}
EIGEN_INC=${EIGEN_INC:-/usr/include/eigen3}
FLAGS=(-std=c++20 -O2 -Wall -I include -I "$EIGEN_INC")
LIBS=(-lyaml-cpp -ltbb)

CORE=(src/Entity.cpp src/Scene.cpp src/Collisions.cpp src/Constraints.cpp
      src/Joints.cpp src/YamlUtils.cpp)

build() {
    local name=$1 main=$2
    echo "==> $name"
    "$CXX" "${FLAGS[@]}" "$main" "${CORE[@]}" "${LIBS[@]}" -o "$OUT/$name"
}

build truck_headless      examples/truck/main_headless.cpp
build joint_control_demo  examples/joint_control_demo/main.cpp

# The test suite is renderer-free too, so it builds the same way. Globbed rather than listed,
# so adding a tests/test_*.cpp file never silently leaves this script behind.
echo "==> fx2d_tests"
TESTS=(tests/main.cpp tests/test_*.cpp)
"$CXX" "${FLAGS[@]}" "${TESTS[@]}" "${CORE[@]}" "${LIBS[@]}" -o "$OUT/fx2d_tests"

echo
echo "Built in $OUT/ — run from the repo root so Scene.yml paths resolve."
