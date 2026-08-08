#!/usr/bin/env bash
# Build the headless Fx2D examples with plain g++ and no graphics dependencies.
#
# Deliberately does NOT reference raylib, Dear ImGui or rlImGui anywhere: if any
# core physics header regains a dependency on the renderer, this script fails.
#
# Requires: g++ (C++20), Eigen 3 headers, yaml-cpp, TBB (for std::execution::par).
#   Debian/Ubuntu: sudo apt install g++ libeigen3-dev libyaml-cpp-dev libtbb-dev
#
# Usage (from the repo root):
#   ./scripts/build_headless.sh
#   ./build-headless/truck_headless
#   ./build-headless/joint_control_demo

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

# The test suite is renderer-free too, so it builds the same way.
echo "==> fx2d_tests"
"$CXX" "${FLAGS[@]}" tests/test_aabb_tree.cpp tests/test_joints.cpp tests/test_ccd.cpp \
    tests/test_capsule_collision.cpp tests/test_collisions_edge.cpp \
    tests/test_angle_precision.cpp tests/test_resting_stability.cpp \
    "${CORE[@]}" "${LIBS[@]}" -o "$OUT/fx2d_tests"

echo
echo "Built in $OUT/ — run from the repo root so Scene.yml paths resolve."
