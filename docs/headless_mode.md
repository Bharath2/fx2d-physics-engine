# Headless Mode

Fx2D can run without a window or renderer — useful for data collection, testing, training ML agents, or batch simulation.

---

## How It Works

`FxScene::step(dt)` advances the physics simulation by `dt` seconds entirely in CPU memory. You can call it in a plain `main()` without ever constructing an `FxRylbRenderer`.

Include `"Fx2D/Physics.h"` rather than `"Fx2D/Core.h"`: it aggregates the math, entity, joint, solver, scene and YAML headers and pulls in **no** raylib, Dear ImGui or rlImGui header. `"Fx2D/Core.h"` is `Physics.h` plus the renderer, so it still requires the full graphics stack.

Headless builds therefore need only:

- a C++20 compiler
- Eigen 3 (header-only) — math types
- yaml-cpp — `Scene.yml` loading
- TBB — backs `std::execution::par` in libstdc++

No GL, X11, Wayland or window system is involved, so this works over SSH, in a container, or in CI.

---

## Building Headless

`scripts/build_headless.sh` compiles the two headless examples with plain `g++` and no reference to any graphics library — if a core header ever regains a renderer dependency, the script fails:

```bash
sudo apt install g++ libeigen3-dev libyaml-cpp-dev libtbb-dev   # Debian/Ubuntu
./scripts/build_headless.sh
./build-headless/truck_headless        # constraint rig: chassis, wheels, ground
./build-headless/joint_control_demo    # revolute + prismatic motor control modes
```

Run the binaries from the repo root — each resolves its `Scene.yml` relative to the working directory.

See `examples/truck/main_headless.cpp` and `examples/joint_control_demo/main.cpp` for the full sources.

---

## Basic Loop

```cpp
#include "Fx2D/Physics.h"
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
scene.set_substeps(14);                    // solver substeps per step() call (default: 14)
scene.set_velocity_passes(4);              // velocity sweeps per substep (default: 4)
scene.set_gravity(FxVec2f(0.0f, -9.81f)); // override gravity
```

Increasing substeps improves constraint and collision accuracy at the cost of CPU time. The
14x4 default is the cheapest configuration measured that still passes the full quality suite;
the two knobs trade against each other, so raise substeps for penetration under load and
velocity passes for stack convergence.

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
