---
title: Demos
description: Six runnable Fx2D example scenes — slingshot, truck, stacked boxes, joint motors, chain terrain and a bucket of bodies — each a short C++ program plus a YAML scene.
---

# Demos

Each example is an ordinary C++ program and `Scene.yml` file in the repository. Build visual demos with `-DFX2D_BUILD_EXAMPLES=ON`, then run binaries from the repository root so asset paths resolve.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DFX2D_BUILD_EXAMPLES=ON
cmake --build build -j
./build/example_angry_boxes
```

Targets: `example_playground`, `example_angry_boxes`, `example_truck`, `example_stacked_boxes`, `example_joint_control`, `example_chain_terrain`, `example_bucket_fill`.

The [playground](/playground) runs in the browser with nothing to build: drag bodies, spawn shapes, reset.

<div class="demo-grid">
  <article class="demo-card" id="angry-boxes">
    <img src="/demos/angry-boxes.gif" alt="Angry boxes: a slingshot ball knocks over a tower in Fx2D" />
    <div>
      <h3><a href="https://github.com/Bharath2/fx2d-physics-engine/tree/main/examples/angry_boxes">Angry boxes</a></h3>
      <p>Drag the ball, release, topple the tower. Mouse input, a trajectory preview drawn through the renderer's draw callback, impact strength read from <code>scene.contacts()</code>, and a reset callback that re-arms the slingshot. The same mechanic runs headlessly in <code>tests/test_slingshot.cpp</code>.</p>
    </div>
  </article>
  <article class="demo-card" id="truck">
    <img src="/demos/truck.gif" alt="A truck drives over a physics scene" />
    <div>
      <h3><a href="https://github.com/Bharath2/fx2d-physics-engine/tree/main/examples/truck">Truck</a></h3>
      <p>A wheeled vehicle that brings suspension, constraints, collision, and textured rendering together. Ships with a headless variant that steps the same scene with no window.</p>
    </div>
  </article>
  <article class="demo-card" id="stacked-boxes">
    <img src="/demos/stacked-boxes.gif" alt="Boxes stack and collide in Fx2D" />
    <div>
      <h3><a href="https://github.com/Bharath2/fx2d-physics-engine/tree/main/examples/stacked_boxes">Stacked boxes</a></h3>
      <p>Basic rigid bodies, textured shapes, gravity, resting stability, and friction. The scene the resting-stability tests are modelled on.</p>
    </div>
  </article>
  <article class="demo-card" id="joint-control">
    <img src="/demos/joint-control.gif" alt="Joint control simulation in Fx2D" />
    <div>
      <h3><a href="https://github.com/Bharath2/fx2d-physics-engine/tree/main/examples/joint_control_demo">Joint control</a></h3>
      <p>Revolute and prismatic motors in position, velocity, and effort control modes with PID tuning. See the <a href="/fx2d-physics-engine/guides/joints">joints guide</a>.</p>
    </div>
  </article>
  <article class="demo-card" id="chain-terrain">
    <img src="/demos/chain-terrain.gif" alt="Balls dropped onto polyline terrain in Fx2D" />
    <div>
      <h3><a href="https://github.com/Bharath2/fx2d-physics-engine/tree/main/examples/chain_terrain">Chain terrain</a></h3>
      <p>Open polyline terrain authored as one chain entity, click-to-spawn bodies at runtime, and one-sided chain contacts with ghost-vertex handling so balls roll smoothly across segment joins.</p>
    </div>
  </article>
  <article class="demo-card" id="bucket-fill">
    <img src="/demos/bucket-fill.gif" alt="Hundreds of bodies pour into a bucket in Fx2D" />
    <div>
      <h3><a href="https://github.com/Bharath2/fx2d-physics-engine/tree/main/examples/bucket_fill">Bucket fill</a></h3>
      <p>Hundreds of bodies piling into a container: the broad phase, contact cache, and sleeping under load. The adversarial test suite runs the same spawn pattern headlessly.</p>
    </div>
  </article>
</div>

For a renderer-free loop suited to testing, simulations, and data collection, see [headless mode](/guides/headless).
