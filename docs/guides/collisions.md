---
title: Collision Setup
description: Configure colliders, contact materials, sensors, filters, and CCD.
---

# Collision setup

Every collidable entity needs a collision shape. Visual geometry is optional and does not participate in physics; configure it separately when using the renderer.

```cpp
auto floor = std::make_shared<FxEntity>("floor");
floor->set_mass(0.0f);  // static: participates in contacts but is not corrected
floor->set_collision_geometry(FxShape(FxVec2f{-8.0f, 0.0f}, FxVec2f{8.0f, 0.0f}));
scene.add_entity(floor);

auto ball = std::make_shared<FxEntity>("ball");
ball->set_mass(1.0f);
ball->set_collision_geometry(FxShape(0.35f));
ball->set_inertia();
ball->set_init_pose(FxVec3f{0.0f, 4.0f, 0.0f});
scene.add_entity(ball);
```

Use an edge or chain for fixed terrain. A zero-skin edge has zero area and inertia, so make it static with zero mass. Dynamic bodies are usually circles, capsules, or convex polygons.

## Tune the contact material

Contact values live on each `FxEntity`. For a pair, restitution uses the more bouncy value and friction uses the lower coefficient—set both bodies deliberately.

```cpp
ball->elasticity = 0.65f;
ball->static_friction = 0.7f;
ball->dynamic_friction = 0.5f;
ball->vel_damping = 0.02f;
```

`elasticity` controls bounce. Static friction resists the start of sliding; dynamic friction applies while sliding. `vel_damping` is a per-body drag-like damping value, not contact friction.

## Observe contacts and make triggers

After a call to `scene.step(dt)`, inspect contacts or begin/end events. A sensor reports overlaps but never exchanges collision impulses.

```cpp
trigger->is_sensor = true;

scene.step(dt);
for (const FxContactEvent& event : scene.begin_contact_events()) {
    // event.entity1 and event.entity2 identify a newly touching pair
}
```

See [contacts and sensors](/guides/events) for the event API and a trigger-focused example.

## Filter pairs intentionally

Entities that share the same negative `collision_group` do not collide with each other. `0` leaves an entity unfiltered.

```cpp
wheel_a->collision_group = -1;
wheel_b->collision_group = -1;  // wheel_a and wheel_b now ignore each other
```

Adding a direct constraint or joint also excludes its connected entity pair from collision while it is registered. This avoids a linked body fighting its own constraint.

## Prevent high-speed tunnelling

Set `enable_ccd` for bodies that could cross a thin collider within one substep. Fx2D then creates speculative contacts for approaching pairs.

```cpp
bullet->enable_ccd = true;
scene.set_substeps(16);
scene.step(1.0 / 60.0);
```

CCD works best with a fixed, reasonably small step and enough substeps. It reduces tunnelling, but it is not a time-of-impact sweep; very fast bodies can still require a smaller frame step or thicker geometry.

For SAT details, contact normals, manifolds, and the exact CCD rules, read the [collision pipeline](/concepts/collisions). The [math and geometry guide](/guides/math) covers every `FxShape` constructor.
