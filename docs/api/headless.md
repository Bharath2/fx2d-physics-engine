---
title: Headless simulation
---

# Headless simulation

Headless mode is the smallest Fx2D integration: no window, no raylib, no ImGui. Build the library with `FX2D_HEADLESS=ON`, include `Scene.h`, and call `step()` yourself.

## Load and step a YAML scene

```cpp
#include "Fx2D/Scene.h"
#include "Fx2D/YamlUtils.h"

#include <iostream>

int main() {
    FxScene scene = FxYAML::buildScene("Scene.yml");
    const auto ball = scene.get_entity("ball");
    if (!ball) return 1;

    constexpr double dt = 1.0 / 120.0;
    for (int step = 0; step != 1200; ++step) {
        scene.step(dt);
    }

    std::cout << ball->pose << '\n';
}
```

`step()` advances the scene, updates contact/event buffers, and invokes a registered `set_step_callback`. Read contacts after the step; their contents remain valid until the next `step()` or `reset()`.

## Build bodies in C++

```cpp
#include "Fx2D/Entity.h"
#include "Fx2D/Scene.h"

#include <memory>

int main() {
    FxScene scene(FxVec2ui{1280, 720});
    scene.set_gravity(FxVec2f{0.0f, -9.81f});

    auto ball = std::make_shared<FxEntity>("ball");
    FxVisualShape visual(0.5f);
    ball->set_visual_geometry(visual);
    ball->set_collision_geometry(FxCollisionShape(0.5f));
    ball->set_init_pose(FxVec3f{0.0f, 5.0f, 0.0f});
    ball->set_mass(1.0f);
    ball->set_inertia();
    scene.add_entity(ball);
    scene.capture_initial_state();

    for (int i = 0; i != 600; ++i) scene.step(1.0 / 120.0);
}
```

Set visual geometry before calling `set_inertia()` when it should define inertia; see [the entity API](/fx2d-physics-engine/cpp-api/classFxEntity.html) for the geometry and force methods.

## Headless input and callbacks

The same callback code runs with and without a renderer. In headless mode, inject input explicitly through `scene.input()` before stepping. The [input guide](/guides/input#headless-injecting-input) documents that producer API; [queries](/guides/queries) and [contact events](/guides/events) work unchanged.
