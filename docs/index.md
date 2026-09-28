---
layout: home
title: Fx2D — 2D rigid-body physics engine in C++20
titleTemplate: false
description: Fx2D is an open-source 2D rigid-body physics engine in C++20 with SAT collision detection, an XPBD constraint solver, motorized joints, YAML scenes, and a headless mode for simulation, testing and reinforcement learning.

hero:
  name: |
    <svg class="home-wordmark" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 340 100" role="img" aria-label="Fx2D" fill="none" stroke="currentColor" stroke-width="24" stroke-linecap="round" stroke-linejoin="round">
      <path d="M12 88V12H60M12 50H50" />
      <path transform="translate(80 0)" d="M12 46L50 88M50 46L12 88" />
      <g class="home-wordmark-accent">
        <path transform="translate(158 0)" d="M13 34C13 20 25 12 39 12C53 12 64 21 64 34C64 46 57 53 46 62L13 88H66" />
        <path transform="translate(250 0)" d="M12 12V88H34C62 88 78 72 78 50C78 28 62 12 34 12Z" />
      </g>
    </svg>
  text: 2D physics in C++
  tagline: Fx2D is an open-source rigid-body physics engine written in C++20. It uses SAT collision detection and XPBD constraints, with YAML scene loading and optional raylib rendering. Licensed under BSD-3-Clause.
  image:
    src: /demos/angry-boxes.gif
    alt: Angry Boxes demo — a slingshot ball topples a tower of boxes simulated by Fx2D
  actions:
    - theme: brand
      text: Browser playground
      link: /playground
    - theme: alt
      text: Build instructions
      link: /getting-started/install
    - theme: alt
      text: Examples
      link: /demos
    - theme: alt
      text: Source on GitHub
      link: https://github.com/Bharath2/fx2d-physics-engine

features:
  - icon: ◒
    title: Collision detection
    details: A dynamic AABB tree finds candidate pairs. Skin-aware SAT generates contacts for circles, capsules, convex polygons, edges, and chains.
  - icon: ≋
    title: Constraint solver
    details: Substepped XPBD with compliance, warm starting, friction, restitution, and motorized revolute and prismatic joints.
  - icon: ↗
    title: Scene queries and input
    details: Ray casts, overlap queries, contact events, sensors, entity groups, and keyboard and mouse input. A mouse joint supports dragging bodies.
  - icon: ▣
    title: Headless simulation
    details: Run the physics library without a window or raylib. Step scenes from C++ and read body state, contacts, and query results.
  - icon: ⧉
    title: Regression tests
    details: Tests cover joints, collisions, contact events, resting stability, and adversarial scenes. Golden-state tests check solver changes; cross-platform bitwise identity is not guaranteed.
  - icon: ≡
    title: YAML scenes
    details: Define bodies, geometry, joints, textures, and solver settings in YAML. Load a scene from C++ and reset it to its authored state.
---

## Browser playground

The [playground](/playground) runs the engine compiled to WebAssembly. You can drag bodies, spawn shapes, and change simulation settings. Its source is the same `examples/playground` program used by the desktop build.

## Running a simulation

Define a scene in YAML, load it, then step it. Use `Fx2D/Core.h` to add the raylib viewer, or work directly with `FxScene` for a headless simulation.

::: code-group

```cpp [With a window]
#include "Fx2D/Core.h"

int main() {
    auto scene = FxYAML::buildScene("Scene.yml");
    FxRylbRenderer renderer(scene, 60);
    renderer.run();
}
```

```cpp [Headless]
#include "Fx2D/Scene.h"
#include "Fx2D/YamlUtils.h"

int main() {
    auto scene = FxYAML::buildScene("Scene.yml");
    auto ball  = scene.get_entity("ball");
    for (int i = 0; i < 600; ++i) {
        scene.step(1.0 / 60.0);
        std::cout << ball->pose << '\n';
    }
}
```

:::

See the [installation guide](/getting-started/install) for dependencies and build commands, and [your first scene](/getting-started/first-scene) for a YAML example.

## Examples

The repository includes C++ programs and YAML scenes for the examples below.

<div class="demo-grid demo-grid--compact">
  <a class="demo-tile" href="/fx2d-physics-engine/demos#angry-boxes"><img src="/demos/angry-boxes.gif" alt="Angry boxes slingshot demo" loading="lazy" /><span>Angry boxes</span></a>
  <a class="demo-tile" href="/fx2d-physics-engine/demos#truck"><img src="/demos/truck.gif" alt="Truck suspension demo" loading="lazy" /><span>Truck</span></a>
  <a class="demo-tile" href="/fx2d-physics-engine/demos#stacked-boxes"><img src="/demos/stacked-boxes.gif" alt="Stacked boxes demo" loading="lazy" /><span>Stacked boxes</span></a>
  <a class="demo-tile" href="/fx2d-physics-engine/demos#joint-control"><img src="/demos/joint-control.gif" alt="Joint motor control demo" loading="lazy" /><span>Joint control</span></a>
  <a class="demo-tile" href="/fx2d-physics-engine/demos#chain-terrain"><img src="/demos/chain-terrain.gif" alt="Chain terrain demo" loading="lazy" /><span>Chain terrain</span></a>
  <a class="demo-tile" href="/fx2d-physics-engine/demos#bucket-fill"><img src="/demos/bucket-fill.gif" alt="Bucket fill demo" loading="lazy" /><span>Bucket fill</span></a>
</div>

## Engine overview

Each simulation step finds potentially colliding pairs, generates contacts, and resolves contact and joint constraints. The [collision detection](/concepts/collisions) and [XPBD solver](/concepts/xpbd) pages describe the algorithms and equations used in the implementation.

| | Fx2D |
|---|---|
| Language | C++20; physics core built as a static library |
| Collision | Dynamic AABB tree + skin-aware SAT, opt-in speculative CCD |
| Solver | Substepped XPBD, warm-started, Coulomb friction, restitution |
| Joints | Revolute and prismatic with position / velocity / effort motors |
| Scenes | YAML with textures, joints, groups, reset |
| Headless | Yes — no raylib, no window, same API |
| Rendering | raylib + Dear ImGui inspector, optional |
| License | BSD-3-Clause |

## Development and contributions

Fx2D is developed in the open. The [roadmap](/roadmap) records pending work, known limitations, and measurements from previous changes. The [contributing guide](/contributing) covers code style, tests, and benchmarks.

Bug reports, reproducible scenes, documentation corrections, and code contributions are welcome on [GitHub](https://github.com/Bharath2/fx2d-physics-engine).
