---
layout: home
title: Fx2D — 2D rigid-body physics engine in C++20
titleTemplate: false
description: Fx2D is an open-source 2D rigid-body physics engine in C++20 with SAT collision detection, an XPBD constraint solver, motorized joints, YAML scenes, and a headless mode for simulation, testing and reinforcement learning.

hero:
  name: Fx2D
  text: Rigid-body physics, made practical.
  tagline: An open-source C++20 engine for responsive 2D worlds — SAT collision detection, XPBD constraints, motorized joints, and a headless mode for simulation and reinforcement learning.
  image:
    src: /demos/angry-boxes.gif
    alt: Angry Boxes demo — a slingshot ball topples a tower of boxes simulated by Fx2D
  actions:
    - theme: brand
      text: Try it in your browser
      link: /playground
    - theme: alt
      text: Get started
      link: /getting-started/install
    - theme: alt
      text: Explore demos
      link: /demos
    - theme: alt
      text: Star on GitHub
      link: https://github.com/Bharath2/fx2d-physics-engine

features:
  - icon: ◒
    title: One shape model
    details: Circles, capsules, polygons, edges, and chains share a skin-aware SAT narrow phase. Rounded shapes cost nothing extra.
  - icon: ≋
    title: Stable constraints
    details: XPBD substeps, compliance, warm starting, friction, and motorized joints keep worlds feeling solid — verified by an adversarial test suite.
  - icon: ↗
    title: Built for interaction
    details: Queries, contacts, sensors, entity groups, keyboard and mouse input, and a click-drag mouse joint are part of the engine, not add-ons.
  - icon: ▣
    title: Headless by design
    details: Step scenes with no window, no GPU, and no raylib. Inject input, read contacts, cast rays, batch rollouts for RL.
  - icon: ⧉
    title: Deterministic
    details: Fixed timestep, fixed solver ordering, single-threaded by default. The same inputs give the same trajectory, every time.
  - icon: ≡
    title: Declarative scenes
    details: Describe bodies, textures, joints and solver settings in YAML, load them in one line, reset at any time.
---

<p class="section-kicker">No install</p>

## Play with it first

The [playground](/playground) is the engine compiled to WebAssembly: drag any body with the mouse, spawn shapes, topple the stack. It is the same `examples/playground` program the desktop build runs.

<p class="section-kicker">Start building</p>

## From scene to simulation

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

The [installation guide](/getting-started/install) gets a project building; [your first scene](/getting-started/first-scene) shows the smallest useful YAML setup.

<p class="section-kicker">See it run</p>

## Six runnable examples

<div class="demo-grid demo-grid--compact">
  <a class="demo-tile" href="/fx2d-physics-engine/demos#angry-boxes"><img src="/demos/angry-boxes.gif" alt="Angry boxes slingshot demo" loading="lazy" /><span>Angry boxes</span></a>
  <a class="demo-tile" href="/fx2d-physics-engine/demos#truck"><img src="/demos/truck.gif" alt="Truck suspension demo" loading="lazy" /><span>Truck</span></a>
  <a class="demo-tile" href="/fx2d-physics-engine/demos#stacked-boxes"><img src="/demos/stacked-boxes.gif" alt="Stacked boxes demo" loading="lazy" /><span>Stacked boxes</span></a>
  <a class="demo-tile" href="/fx2d-physics-engine/demos#joint-control"><img src="/demos/joint-control.gif" alt="Joint motor control demo" loading="lazy" /><span>Joint control</span></a>
  <a class="demo-tile" href="/fx2d-physics-engine/demos#chain-terrain"><img src="/demos/chain-terrain.gif" alt="Chain terrain demo" loading="lazy" /><span>Chain terrain</span></a>
  <a class="demo-tile" href="/fx2d-physics-engine/demos#bucket-fill"><img src="/demos/bucket-fill.gif" alt="Bucket fill demo" loading="lazy" /><span>Bucket fill</span></a>
</div>

<p class="section-kicker">Explore the engine</p>

## Physics you can inspect

Fx2D has an open, explicit pipeline: a dynamic AABB tree proposes pairs, SAT produces contacts across the unified shape representation, then an XPBD solver resolves constraints over substeps. Dive into [collision detection](/concepts/collisions) or the [solver](/concepts/xpbd), or go straight to the runnable [demos](/demos).

| | Fx2D |
|---|---|
| Language | C++20, ~7k lines, one static library |
| Collision | Dynamic AABB tree + skin-aware SAT, opt-in speculative CCD |
| Solver | Substepped XPBD, warm-started, Coulomb friction, restitution |
| Joints | Revolute and prismatic with position / velocity / effort motors |
| Scenes | YAML with textures, joints, groups, reset |
| Headless | Yes — no raylib, no window, same API |
| Rendering | raylib + Dear ImGui inspector, optional |
| License | BSD-3-Clause |

## Who is it for?

- **Game developers** who want a readable 2D engine they can step through in a debugger and tune with real parameters.
- **Researchers and students** who want the equations and the code side by side: the [XPBD](/concepts/xpbd) and [collision](/concepts/collisions) pages derive what the source implements.
- **Reinforcement-learning and robotics work** that needs a deterministic, headless environment with scripted input, contacts and ray casts as observations.

Read the [roadmap](/roadmap) to see what is next and the measurements behind each decision, or [contribute](/contributing).
