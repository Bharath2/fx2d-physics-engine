---
title: Renderer integration
---

# Renderer integration

`FxRylbRenderer` owns the visual loop around an `FxScene`: it opens the raylib window, advances physics at a fixed timestep, synchronizes input, draws shapes/textures, and shows the ImGui controls.

## Minimal viewer

```cpp
#include "Fx2D/Core.h"

int main() {
    FxScene scene = FxYAML::buildScene("Scene.yml");
    FxRylbRenderer renderer(scene, 60);
    renderer.run();
}
```

Run the executable from a working directory where the paths used by `Scene.yml` resolve. Repository demos are run from the repository root for this reason.

## Add application behavior

```cpp
scene.set_step_callback([](FxScene& scene, double) {
    if (scene.input().key_pressed(FxKey::Space)) {
        if (auto ball = scene.get_entity("ball")) {
            ball->apply_impulse(FxVec2f{0.0f, 8.0f});
        }
    }
});

FxRylbRenderer renderer(scene, 60);
renderer.set_draw_callback([](FxRylbRenderer& renderer) {
    // Draw overlays here; use renderer.world_to_screen() for scene coordinates.
});
renderer.run();
```

The renderer gives ImGui first refusal over input, so UI clicks do not accidentally control the scene. Read [input](/guides/input) for edge-event timing and [the renderer guide](/guides/renderer) for backgrounds, camera transforms, and real-time factor.

## Required build mode

This route requires `FX2D_HEADLESS=OFF`, raylib, yaml-cpp, Eigen3, Dear ImGui, and rlImGui. For CMake consumption instructions, use [link Fx2Dlib](/api/linking#enable-the-renderer).
