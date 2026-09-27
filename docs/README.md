# Fx2D documentation

These pages are the source of the [documentation site](https://bharath2.github.io/fx2d-physics-engine/), which adds the browser playground, runnable demos and the generated C++ API reference. Every page also reads fine here on GitHub.

## Getting started

- [Installation](getting-started/install.md): dependencies and build steps for Linux, macOS, Windows and MSYS2
- [Your first scene](getting-started/first-scene.md): write a YAML scene and step it from C++

## Guides

- [Build with Fx2D](guides/index.md): an overview of the guides
- [Scene authoring](guides/scene-authoring.md): build a world in YAML, from simulation settings to entities and joints
- [Collision setup](guides/collisions.md): colliders, contact materials, sensors, filters and CCD
- [Constraints](guides/constraints.md): connect bodies with XPBD constraints or motorized joints
- [Joint control](guides/joints.md): motor modes, PID tuning and the mouse joint
- [Entity groups](guides/entity-groups.md): named sets of entities, intra-group collision filtering and reset
- [Contacts, events and sensors](guides/events.md): read each step's contacts and begin/end events
- [Spatial queries](guides/queries.md): ray casts, overlap and point queries
- [Keyboard and mouse input](guides/input.md): gameplay input, windowed or injected headlessly
- [Headless mode](guides/headless.md): run without a renderer for tests, batch simulation and RL
- [Renderer](guides/renderer.md): the raylib viewer, real-time factor, ImGui panel and textures
- [Math and geometry](guides/math.md): vectors, poses, transforms and collision shapes

## Concepts

- [XPBD solver](concepts/xpbd.md): the per-substep pipeline and the constraint equations
- [Collision resolution](concepts/collisions.md): broad phase, SAT narrow phase, restitution and friction

## Reference

- [Scene YAML](reference/scene-yaml.md): every scene block, entity field, geometry type and joint
- [Math utilities](reference/math.md): `FxArray`, vector and matrix types, and helper functions
- [C++ API overview](api-overview.md): the entry page of the generated API reference

## Project

- [Roadmap](roadmap.md): what comes next, with the measurements behind each decision
- [SIMD plan](roadmap/simd.md) and [Fx3D plan](roadmap/fx3d.md): longer-range design notes
- [Next steps](next_steps.md): handoff notes on where step time goes and what to pick up
- [Contributing](contributing.md): workflow, lint gate, tests and measurement rules
- [Demos](demos.md) and [Playground](playground.md): the example gallery and the in-browser sandbox
