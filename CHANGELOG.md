# Changelog

Notable changes to Fx2D, newest first. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/): while the version is 0.x, a minor version may break
compatibility, and every such change is listed here.

## [Unreleased]

## [0.1.0] - 2026-11-10

The first release, and the first on PyPI (`pip install fx2d`). It covers the development since
the repository opened in June 2025.

### Added

- **Engine.** A C++20 static library: a dynamic AABB tree broad phase, SAT contacts with clipped
  manifolds, and a substepped XPBD solver with friction, restitution, warm starting, sleeping and
  a vectorised contact solve (AVX2 and NEON from the same source).
- **Shapes.** Circles, capsules, edges, chains for terrain, and convex polygons, any of them
  rounded through one skin-radius model.
- **Joints.** Revolute, prismatic, distance and rope joints with PID motors in position,
  velocity and effort modes, and a mouse joint for dragging bodies.
- **Collisions held through the substeps.** Detection runs once per step and each contact is
  re-measured in the later substeps, after Macklin et al., *Small Steps in Physics Simulation*.
- **Opt-in speculative CCD** for fast bodies.
- **Scenes.** YAML scenes with textures and joints, entity groups with collision filtering, and
  one-call reset.
- **Queries and events.** Ray casts, overlap and point queries, begin and end contact events,
  and sensors.
- **Input** that works in the viewer and headless, for driving scenes from code.
- **Drag.** `vel_damping` on bodies.
- **Viewer and examples.** A raylib viewer with an ImGui inspector; the playground, angry boxes,
  truck, stacked boxes, joint control, chain terrain and bucket fill examples; and a browser
  playground built with WebAssembly.
- **Python package.** nanobind bindings for the whole scene builder (every shape, joint, group,
  query and event), with bodies and joints held by name and shared with the C++ objects; NumPy
  views of a scene's state; and `fx2d.step_many`, which steps independent scenes in parallel with
  the GIL released.
- **Gymnasium environments.** `fx2d/Push-v0`, `fx2d/CartPole-v0` and `fx2d/Pendulum-v0`, each
  with a vector version for `gym.make_vec` that steps every copy in one parallel call.
- **Wheels** for CPython 3.10-3.14 on Linux (x86-64 and ARM64), macOS and Windows, built,
  tested and published by CI.
- **Benchmarks.** A step benchmark, a comparison with Box2D v3, and an environment throughput
  benchmark.
- **Tests.** Suites covering collisions, joints, contacts, queries, resting stability and
  adversarial scenes, and bitwise golden tests of the simulated state.

### Changed

- **Results are bit-identical across platforms.** The same scene and inputs give the same bits
  with GCC, Clang, Apple Clang and MSVC, on x86-64 and ARM64, checked by the golden tests in CI.
  Fx2D computes its own sine and cosine and turns off floating-point contraction.
- **No required dependencies.** Fx2D's own vector and matrix types replaced Eigen, with
  bit-identical results. yaml-cpp, raylib and Dear ImGui are needed only for the YAML loader and
  the viewer, and CMake downloads them when they are not installed.
- **Solver defaults.** 14 substeps with 4 velocity passes each.
- **Default elasticity** is 0.1, down from 1.0. Restitution mixes by maximum and friction by
  minimum.
- **Applied forces.** A force or torque applied between steps acts for the whole of the next
  step. It used to act for one substep in 14, so code tuned around the old behaviour now pushes
  about 14 times harder.

### Fixed

- A box landing on a corner comes to rest instead of rocking without end.
- Capsules lying on a face, or on another capsule, rest on two contact points instead of rocking
  from end to end.
- Edges authored away from their body's origin collide on both sides.
- `vel_damping` is applied; it used to be read from YAML and ignored.
- Resting contacts no longer gain energy, and small rotations are no longer rounded away.
- `reset()` reloads the scene completely, inertia from YAML comes from the authored shape, and
  chains hold on steep terrain and collide on one side only.

[Unreleased]: https://github.com/Bharath2/fx2d-physics-engine/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/Bharath2/fx2d-physics-engine/releases/tag/v0.1.0
