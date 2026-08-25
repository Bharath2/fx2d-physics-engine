---
layout: home

hero:
  name: Fx2D
  text: Rigid-body physics, made practical.
  tagline: A C++20 engine for responsive 2D worlds — SAT collision detection, XPBD constraints, and a renderer when you want one.
  image:
    src: /demos/stacked-boxes.gif
    alt: Stacked boxes simulated by Fx2D
  actions:
    - theme: brand
      text: Get started
      link: /getting-started/install
    - theme: alt
      text: Explore demos
      link: /demos
    - theme: alt
      text: Browse the API
      link: /api/

features:
  - icon: ◒
    title: One shape model
    details: Circles, capsules, polygons, edges, and chains share a skin-aware SAT narrow phase.
  - icon: ≋
    title: Stable constraints
    details: XPBD substeps, compliance, warm starting, friction, and motorized joints keep worlds feeling solid.
  - icon: ↗
    title: Built for interaction
    details: Queries, contacts, sensors, entity groups, keyboard and mouse input are part of the engine—not add-ons.
---

<p class="section-kicker">Start building</p>

## From scene to simulation

Define a scene in YAML, load it, then step it. Use `Fx2D/Core.h` to add the raylib viewer, or work directly with `FxScene` for a headless simulation.

```cpp
#include "Fx2D/Core.h"

int main() {
    auto scene = FxYAML::buildScene("Scene.yml");
    FxRylbRenderer renderer(scene, 60);
    renderer.run();
}
```

The [installation guide](/getting-started/install) gets a project building; [your first scene](/getting-started/first-scene) shows the smallest useful YAML setup.

<p class="section-kicker">Explore the engine</p>

## Physics you can inspect

Fx2D has an open, explicit pipeline: a dynamic AABB tree proposes pairs, SAT produces contacts across the unified shape representation, then an XPBD solver resolves constraints over substeps. Dive into [collision detection](/concepts/collisions) or the [solver](/concepts/xpbd), or go straight to the runnable [demos](/demos).
