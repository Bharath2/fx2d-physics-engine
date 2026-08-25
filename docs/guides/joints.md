# Joint Control Reference

Fx2D joints connect two entities and expose a motor API built on a shared PID controller. Both joint types (`FxRevoluteJoint`, `FxPrismaticJoint`) inherit the same base interface and differ only in the physical quantity they control (angle vs. translation).

Include via:

```cpp
#include "Fx2D/Core.h"
```

---

## Accessing Joints

Joints are looked up by name from a scene:

```cpp
auto joint = scene.get_joint("wheel_hinge");              // returns shared_ptr<FxJoint>, nullptr if missing
auto rev   = std::dynamic_pointer_cast<FxRevoluteJoint>(joint);
auto pri   = std::dynamic_pointer_cast<FxPrismaticJoint>(joint);
```

Other scene-level helpers:

```cpp
scene.joint_exists("wheel_hinge");   // bool
scene.joint_count();                 // size_t
scene.delete_joint("wheel_hinge");   // bool – removes joint and its constraints
```

---

## Control Modes

Every joint has one active `ControlMode`:

| Mode | Enum | `FxRevoluteJoint` target unit | `FxPrismaticJoint` target unit |
|---|---|---|---|
| `POSITION` | `ControlMode::POSITION` | angle (radians) | translation along axis |
| `VELOCITY` | `ControlMode::VELOCITY` | angular velocity (rad/s) | linear velocity |
| `EFFORT`   | `ControlMode::EFFORT`   | torque | force |

Switching mode resets the PID integral and derivative accumulators automatically.

```cpp
joint->set_control_mode(ControlMode::VELOCITY);
joint->get_control_mode();   // ControlMode
```

---

## Revolute Joint (`FxRevoluteJoint`)

Controls the relative angle between two bodies around a shared anchor point.

### Setting targets

```cpp
// Position mode — drive to an angle
rev->set_control_mode(ControlMode::POSITION);
rev->set_theta(0.5f);             // target 0.5 rad; PID drives there over time
rev->set_theta(0.5f, true);       // also snaps entities immediately (instant correction)

// Velocity mode — spin at a constant rate
rev->set_control_mode(ControlMode::VELOCITY);
rev->set_omega(3.14f);            // target ~180°/s

// Effort mode — apply a fixed torque each step
rev->set_control_mode(ControlMode::EFFORT);
rev->set_torque(15.0f);           // 15 N·m
```

### Reading state

```cpp
float angle = rev->get_theta();   // current relative angle in radians
float omega = rev->get_omega();   // current relative angular velocity in rad/s
```

### Torque limit

```cpp
rev->set_max_torque(20.0f);       // clamps motor output; alias for set_max_effort()
float limit = rev->get_max_torque();
```

---

## Prismatic Joint (`FxPrismaticJoint`)

Controls the relative translation of two bodies along a locked axis.

### Setting targets

```cpp
// Position mode — move to a point along the axis
pri->set_control_mode(ControlMode::POSITION);
pri->set_position(1.5f);          // target offset 1.5 units from initial distance
pri->set_position(1.5f, true);    // also snaps entities immediately

// Velocity mode — slide at a constant speed
pri->set_control_mode(ControlMode::VELOCITY);
pri->set_velocity(2.0f);          // target 2 units/s along axis

// Effort mode — apply a fixed force each step
pri->set_control_mode(ControlMode::EFFORT);
pri->set_force(8.0f);             // 8 N along axis
```

### Reading state

```cpp
float pos = pri->get_position();  // current translation relative to initial distance
float vel = pri->get_velocity();  // current velocity along axis
```

### Force limit

```cpp
pri->set_max_force(10.0f);        // alias for set_max_effort()
float limit = pri->get_max_force();
```

---

## PID Tuning

All motor modes except `EFFORT` route their error signal through a PID controller before applying the result as effort.

```cpp
joint->set_pid({5.0f, 0.1f, 0.2f});   // set P, I, D gains at once (resets state)
joint->set_p(5.0f);
joint->set_i(0.1f);
joint->set_d(0.2f);

FxVec3f gains = joint->get_pid();      // {p, i, d}
```

`set_pid()` always resets the integral and previous-error accumulators. Calling `set_p/i/d` individually does **not** reset state — useful for live tuning.

### `instant` mode

By default (`m_instant = true`), `set_theta`, `set_omega`, `set_position`, and `set_velocity` also apply an immediate pose/velocity correction on top of the PID output. Disable this for smoother, purely PID-driven motion:

```cpp
joint->set_instant(false);
```

Or override per call:

```cpp
rev->set_theta(0.5f, /*instant=*/false);
```

---

## Shared Base API

These apply to both joint types:

```cpp
joint->enabled = false;                    // disable motor (constraints still active)
joint->entities_collide = true;            // allow parent/child collision

joint->set_max_effort(20.0f);             // universal effort cap
float cap = joint->get_max_effort();

joint->get_name();                        // const string&
joint->get_entity1();                     // shared_ptr<FxEntity> — parent
joint->get_entity2();                     // shared_ptr<FxEntity> — child
joint->is_revolute();                     // bool
joint->is_prismatic();                    // bool
```

---

## Creating Joints in C++

Joints can be created directly without YAML:

```cpp
auto chassis = scene.get_entity("chassis");
auto wheel   = scene.get_entity("wheel");

// Revolute: anchor in chassis local frame, limits ±0.6 rad
auto hinge = std::make_shared<FxRevoluteJoint>(
    "wheel_hinge", chassis, wheel,
    FxVec2f{0.0f, -0.5f},   // anchor point
    -0.6f, 0.6f              // angle_min, angle_max
);
hinge->set_pid({5.0f, 0.2f, 0.1f});
hinge->set_max_torque(20.0f);
hinge->set_control_mode(ControlMode::EFFORT);
hinge->set_torque(12.0f);
scene.add_joint(hinge);

// Prismatic: slide along X axis, limits −2 to +2
auto rail     = scene.get_entity("rail");
auto carriage = scene.get_entity("carriage");
auto slider = std::make_shared<FxPrismaticJoint>(
    "slider", rail, carriage,
    FxVec2f{1.0f, 0.0f},    // axis
    -2.0f, 2.0f              // position_min, position_max
);
slider->set_pid({4.0f, 0.0f, 0.2f});
slider->set_max_force(8.0f);
slider->set_control_mode(ControlMode::VELOCITY);
slider->set_velocity(1.5f);
scene.add_joint(slider);
```

For YAML-based joint configuration see [Scene YAML](../reference/scene-yaml).

---

## Runnable Example

[The joint-control demo](https://github.com/Bharath2/fx2d-physics-engine/tree/main/examples/joint_control_demo) is a complete headless demo. Its `Scene.yml` declares an `arm_motor` (revolute) and a `slider_motor` (prismatic), and `main.cpp` drives both through `POSITION`, `VELOCITY`, and `EFFORT` phases, printing angle, angular velocity, slider offset, and slider velocity as they track. Swap the step loops for `FxRylbRenderer(scene, 60).run()` to watch it instead.

Two things the example demonstrates that are easy to get wrong:

**Position and velocity loops need different gains.** A position loop wants strong damping — roughly `D = 2*sqrt(P * inertia)` for a revolute joint, or `2*sqrt(P * mass)` for a prismatic one — or the joint oscillates around the target. In velocity mode the error is already a rate, so reusing that `D` differentiates acceleration and fights the motor hard enough to stall it. The example calls `set_pid()` at the start of each phase.

**Very small timesteps weaken offset-pivot motors.** Per-substep rotations shrink quadratically with the timestep, and constraint corrections are computed on `float` world coordinates. Once a correction falls below one ulp of the coordinate magnitude — about `4.8e-7` at `x = 7` — it is lost to rounding. A revolute joint whose anchor is offset from the child's centre of mass depends on exactly those small displacements, so at `dt = 1e-3` such a motor loses most of its authority, and the effect worsens the further the scene sits from the origin. The same joint placed near the origin rotates roughly five orders of magnitude further per unit time.

The example runs at `dt = 0.01`, where the effect is negligible. If a motor seems weak or dead, try a larger timestep, or move the scene closer to the origin, before re-tuning gains. Anchoring the joint at the child's centre of mass also avoids it, since rotation about the centroid does not displace the anchor.

Note that a related but distinct bug — `FxAngleWrap` rounding away any rotation under `1.2e-7` rad, which froze *all* angular motion at small timesteps — has been fixed. Free bodies and centre-anchored joints now rotate correctly at `dt = 1e-3`.

## Constraint naming

A joint's constraints are registered as `<jointname>_<Type>` — `lift_Anchor`,
`bridge_j3_AngleLmt` — with `_N` appended only when one joint owns several constraints of the
same type. Joint names are unique in the registry, so constraint names are unique by
construction, and they survive entity renames because the entity pair is not part of the name.
The YAML `joints:` section is name-keyed, so authored joint names flow straight through.
Standalone constraints added directly with `add_constraint` keep their `e1_e2_Type` names.
