---
title: Constraints
description: Connect bodies with direct XPBD constraints or motorized joints.
---

# Constraints

Use a **joint** when you need a common mechanical connection with motor control. Use a **direct constraint** when you want to compose exactly which positional degrees of freedom are restricted.

| Goal | Preferred tool |
| --- | --- |
| Hinge or slider with a position, velocity, or effort motor | [`FxRevoluteJoint` or `FxPrismaticJoint`](/guides/joints) |
| Keep two local points together | `FxAnchorConstraint` |
| Lock a relative angle | `FxAngleLockConstraint` |
| Restrict a relative angle to a range | `FxAngularLimitConstraint` |
| Limit separation along one axis | `FxSeparationConstraint` |
| Keep motion on one axis | `FxMotionAlongAxisConstraint` |

## Compose a slider

A useful direct composition is a slider: one constraint prevents sideways motion and another limits travel along the slide axis.

```cpp
#include "Fx2D/Scene.h"

const FxVec2f axis{1.0f, 0.0f};  // non-zero; local to rail by default

auto keep_on_rail = std::make_shared<FxMotionAlongAxisConstraint>(rail, carriage, axis);
auto travel_limit = std::make_shared<FxSeparationConstraint>(rail, carriage, axis);
travel_limit->lower_limit = -2.0f;
travel_limit->upper_limit =  2.0f;

keep_on_rail->set_stiffness(1.0e7);
travel_limit->set_stiffness(1.0e7);

scene.add_constraint(keep_on_rail);
scene.add_constraint(travel_limit);
```

The axis is normalized by the constructor. By default it is local to `entity1` (the rail), so it rotates with that entity. Pass `false` as the fourth argument to keep the axis in world space.

## Build a hinge from primitives

An anchor makes two local points coincide. Pair it with an angular limit or angle lock when the relative orientation matters.

```cpp
auto pivot = std::make_shared<FxAnchorConstraint>(base, arm, FxVec2f{0.0f, 0.6f});
auto stop = std::make_shared<FxAngularLimitConstraint>(base, arm);
stop->lower_limit = -0.75f;  // radians
stop->upper_limit =  0.75f;

scene.add_constraint(pivot);
scene.add_constraint(stop);
```

The anchor is local to the first entity by default; set the final constructor argument to `false` when supplying a world-space anchor. Angular values are radians, despite older source comments that mention degrees.

## Softness, limits, and registration

Constraints are solved by the XPBD solver once per substep. `set_stiffness(k)` converts a positive stiffness to compliance (`1 / k`); larger values are stiffer. Use `setCompliance(c)` when you need to set XPBD compliance directly.

```cpp
auto lock = std::make_shared<FxAngleLockConstraint>(base, arm, 0.0f);
lock->setCompliance(1.0e-6);  // softer than a very stiff joint
scene.add_constraint(lock);
```

`scene.add_constraint()` registers the constraint and disables collision between its two entities. `delete_constraint(constraint->get_name())` removes it and restores that pair's normal collision eligibility. Names are derived from the two entity names and the constraint type, so only add one direct constraint of each type for the same pair; use a named joint when you need a reusable compound connection.

For solver mathematics, substeps, and the full list of constraint equations, read the [XPBD solver guide](/concepts/xpbd). For a motorized revolute or prismatic connection, use the [joints and motors guide](/guides/joints).
