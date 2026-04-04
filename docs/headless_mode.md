# Headless Mode

Fx2D can run without a window or renderer — useful for data collection, testing, training ML agents, or batch simulation.

---

## How It Works

`FxScene::step(dt)` advances the physics simulation by `dt` seconds entirely in CPU memory. There is no dependency on raylib or a display context. You can call it in a plain `main()` without ever constructing an `FxRylbRenderer`.

---

## Basic Loop

```cpp
#include "Fx2D/Core.h"
#include <iostream>

int main() {
    auto scene = FxYAML::buildScene("./Scene.yml");

    const double dt = 0.001;  // 1 ms fixed time step
    auto ball = scene.get_entity("ball");

    for (size_t i = 0; i < 10000; ++i) {
        scene.step(dt);
        std::cout << ball->pose.transpose() << "\n";  // x, y, theta
    }
}
```

`pose` is an `FxVec3f` where `(x, y)` is position and `z()` / `theta()` is orientation in radians.

---

## Step Callback

To execute custom logic after every physics step (e.g. logging, applying forces, checking termination conditions), register a callback before running:

```cpp
scene.set_step_callback([](FxScene& s, double dt) {
    auto ball = s.get_entity("ball");
    // apply a force, log state, etc.
});
```

The callback receives the scene by reference and the `dt` that was just applied.

---

## Time Tracking

```cpp
double t = scene.time_elapsed();  // total simulation time in seconds since start
```

---

## Simulation Parameters

```cpp
scene.set_substeps(11);                    // solver substeps per step() call (default: 11)
scene.set_gravity(FxVec2f(0.0f, -9.81f)); // override gravity
```

Increasing substeps improves constraint and collision accuracy at the cost of CPU time.

---

## Resetting the Scene

```cpp
scene.reset();  // resets all entities to their initial poses and velocities
```

---

## Step Constraints

`FxScene::step(dt)` clamps `dt` internally to `[1e-3, 0.06]` seconds:

- `dt < 1e-3` — throws `std::invalid_argument` (too small, simulation would be unstable)
- `dt > 0.06` — silently clamped down to `0.06`

---

## Dynamic Entity Management

Entities, constraints, and joints can be added or removed at any point during the headless loop:

```cpp
bool ok = scene.add_entity(my_entity);      // false if name already exists
bool ok = scene.delete_entity("ball");      // false if not found
auto ptr = scene.get_entity("ball");        // nullptr if not found

scene.add_constraint(my_constraint);
scene.delete_constraint("spring1");

scene.add_joint(my_joint);
scene.delete_joint("hinge1");
```

---

## Collision Control

By default all entity pairs participate in collision. You can selectively disable pairs:

```cpp
scene.disable_collision("bodyA", "bodyB");
scene.enable_collision("bodyA", "bodyB");   // re-enable
```

---

## Accessing Entity State

```cpp
auto e = scene.get_entity("box");

e->pose;             // FxVec3f: (x, y, theta_radians)
e->velocity;         // FxVec3f: (vx, vy, omega)
e->prev_pose;        // FxVec3f: pose from previous step
e->prev_velocity;    // FxVec3f: velocity from previous step
e->mass();           // float — getter (not a field)
e->inertia();        // float — getter (not a field)
e->inv_mass();       // float
e->inv_inertia();    // float
e->enabled;          // bool — if false, entity is skipped in physics, collisions, and rendering
e->elasticity;       // float
e->vel_damping;      // float
e->gravity_scale;    // float
e->static_friction;  // float
e->dynamic_friction; // float
```
