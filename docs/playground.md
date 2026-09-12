---
title: Playground
description: Try the Fx2D physics engine in your browser. Drag bodies with the mouse, spawn shapes, knock over the stack. Nothing to install.
---

# Playground

The engine, compiled to WebAssembly, running in your browser. **Drag anything** with the left mouse button. **Right-click** or press **Space** to spawn a shape at the cursor, **C** clears what you spawned, **R** resets the scene.

<div class="playground-frame">
  <iframe src="/fx2d-physics-engine/playground/index.html" title="Fx2D playground" allow="fullscreen" loading="eager"></iframe>
</div>

What you are dragging is the [mouse joint](/guides/joints#mouse-joint): a damped spring from the cursor to the point you grabbed, solved by the same XPBD kernel as every other constraint. The seesaw is a revolute joint, the hillside is a chain collider, and the rounded hexagon is a polygon with a skin radius.

## The same program on the desktop

The playground is [`examples/playground`](https://github.com/Bharath2/fx2d-physics-engine/tree/main/examples/playground), an ordinary example: one `main.cpp`, one `Scene.yml`. The renderer's frame loop is the only thing Emscripten changes, and it does that through `emscripten_set_main_loop`.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DFX2D_BUILD_EXAMPLES=ON
cmake --build build -j --target example_playground
./build/example_playground
```

## Building it for the browser

With an activated [emsdk](https://emscripten.org/docs/getting_started/downloads.html) on the path, one script fetches raylib, yaml-cpp, Eigen, Dear ImGui and rlImGui, compiles them for the web, builds the playground, and copies the result into `docs/public/playground/`:

```bash
./scripts/build_web.sh
```

The output is three files, `playground.js`, `playground.wasm` and `playground.data` (the preloaded scene), served next to a small `index.html`. Any Fx2D program that uses `FxRylbRenderer::run()` can be built the same way; add it as a target under `FX2D_BUILD_WEB` in `CMakeLists.txt`.
