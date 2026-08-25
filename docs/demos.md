---
title: Demos
description: Runnable Fx2D scenes that demonstrate the engine's core systems.
---

# Demos

Each example is an ordinary C++ program and `Scene.yml` file in the repository. Build visual demos with `-DFX2D_BUILD_EXAMPLES=ON`, then run binaries from the repository root so asset paths resolve.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DFX2D_BUILD_EXAMPLES=ON
cmake --build build -j --target example_stacked_boxes example_truck example_joint_control
./build/example_stacked_boxes
```

<div class="demo-grid">
  <article class="demo-card">
    <img src="/demos/stacked-boxes.gif" alt="Boxes stack and collide in Fx2D" />
    <div>
      <h3><a href="https://github.com/Bharath2/fx2d-physics-engine/tree/main/examples/stacked_boxes">Stacked boxes</a></h3>
      <p>Basic rigid bodies, textured shapes, gravity, resting stability, and friction.</p>
    </div>
  </article>
  <article class="demo-card">
    <img src="/demos/truck.gif" alt="A truck drives over a physics scene" />
    <div>
      <h3><a href="https://github.com/Bharath2/fx2d-physics-engine/tree/main/examples/truck">Truck</a></h3>
      <p>A wheeled vehicle that brings suspension, constraints, collision, and rendering together.</p>
    </div>
  </article>
  <article class="demo-card">
    <img src="/demos/joint-control.gif" alt="Joint control simulation in Fx2D" />
    <div>
      <h3><a href="https://github.com/Bharath2/fx2d-physics-engine/tree/main/examples/joint_control_demo">Joint control</a></h3>
      <p>Revolute and prismatic motors in position, velocity, and effort control modes.</p>
    </div>
  </article>
</div>

## More runnable scenes

| Demo | What it covers |
|---|---|
| [Angry boxes](https://github.com/Bharath2/fx2d-physics-engine/tree/main/examples/angry_boxes) | Mouse-driven slingshot input, collisions, and a destructible tower. |
| [Chain terrain](https://github.com/Bharath2/fx2d-physics-engine/tree/main/examples/chain_terrain) | Open polyline terrain, click-to-spawn, and chain colliders. |
| [Bucket fill](https://github.com/Bharath2/fx2d-physics-engine/tree/main/examples/bucket_fill) | A dense particle-style rigid-body scene with a container. |

For a renderer-free loop suited to testing, simulations, and data collection, see [headless mode](/guides/headless).
