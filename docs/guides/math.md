---
title: Math and Geometry
description: Use Fx2D vectors, poses, transforms, and collision shapes safely.
---

# Math and geometry

Fx2D's public math types are small extensions of Eigen types. Use `FxVec2f` for 2D values, and `FxVec3f` for a body's `(x, y, theta)` pose or `(vx, vy, omega)` velocity.

```cpp
#include "Fx2D/Scene.h"  // brings in math, geometry, entities, and the solver

FxVec2f force{20.0f, 0.0f};
FxVec3f pose{2.0f, 1.0f, FxPif / 4.0f};
```

## Keep angle units straight

Engine pose angles, angular velocity, joint angles, and `FxAngleWrap()` use **radians**. The vector helper named `rotate()` is the exception: it takes degrees for convenience. Prefer the explicit `_rad` version in physics code.

```cpp
FxVec2f heading{1.0f, 0.0f};
FxVec2f quarter_turn = heading.rotate_rad(FxPif / 2.0f);
FxVec2f screen_turn  = heading.rotate(90.0f);  // degrees

float relative = FxAngleWrap(body_b->pose.theta() - body_a->pose.theta());
```

`FxVec2f` also supplies `perp()`, `perpCW()`, and a scalar 2D `cross()`. The inherited Eigen operations such as `.dot()`, `.norm()`, and `.normalized()` are available too.

## Move between body and world space

Use the entity transform helpers instead of applying the pose manually. They account for both translation and rotation.

```cpp
const FxVec2f local_thruster{0.4f, 0.0f};
const FxVec2f world_thruster = ship->to_world_frame(local_thruster);

ship->apply_force(FxVec2f{0.0f, 30.0f}, world_thruster);

const FxVec2f local_hit = ship->to_entity_frame(world_hit);
```

`FxVec3f::xy()` returns the position part of a pose; `theta()` is an alias for `z()`. Use `get_xy()` when you explicitly want a copy.

## Build collision geometry

`FxShape` is the one shape type used for both collision and visual geometry. A shape is local to its entity; it follows the entity automatically after `set_collision_geometry()`.

```cpp
auto crate = std::make_shared<FxEntity>("crate");
crate->set_mass(2.0f);
crate->set_collision_geometry(FxShape(FxVec2f{1.2f, 0.8f}, 0.05f)); // rounded box
crate->set_inertia();
crate->set_init_pose(FxVec3f{0.0f, 3.0f, 0.0f});
scene.add_entity(crate);
```

Choose the constructor by the geometry you need:

| Geometry | Constructor |
| --- | --- |
| Circle | `FxShape(radius)` |
| Capsule | `FxShape(length, radius)` |
| Edge | `FxShape(point_a, point_b)` |
| Chain terrain | `FxShape::make_chain(points)` |
| Convex polygon | `FxShape(vertices, skin_radius)` |
| Rectangle | `FxShape(size, skin_radius)` |

Polygons must be convex. Rectangle and polygon vertices are recentered on construction; edges and chains preserve their authored points, making them suitable for static level geometry. Call `set_inertia()` after setting the collision geometry when the shape should determine the body's inertia.

## Arrays for geometry batches

`FxVec2fArray` is an aligned `FxArray<FxVec2f>`, useful for polygon and chain points. It supports range-for iteration and vector operations across the whole array.

```cpp
FxVec2fArray terrain = {
  {-8.0f, 0.0f}, {-2.0f, 0.5f}, {2.0f, -0.2f}, {8.0f, 1.0f},
};

ground->set_collision_geometry(FxShape::make_chain(terrain));
```

For every available helper, matrix type, and constructor overload, use the [math reference](/reference/math) or the generated [`Math.h` symbols](/cpp-api/Math_8h.html).
