---
title: Scene Authoring
description: Build an Fx2D world in YAML, from simulation settings through entities and joints.
---

# Scene authoring

Use YAML when you want an editable scene description separate from your application code. An Fx2D scene has three practical layers: simulation settings, named entities, and optional joints.

```yaml
scene:
  size: [16, 9]
  gravity: [0, -9.81]

entities:
  floor:
    pose: [8, 0.5, 0]
    physics:
      mass: 0
      gravity_scale: 0
    collision:
      geometry:
        rectangle: [16, 1]

  ball:
    pose: [8, 7, 0]
    physics:
      mass: 1.0
      elasticity: 0.5
      static_friction: 0.7
      dynamic_friction: 0.5
    collision:
      geometry:
        circle: 0.25
```

Load that scene with `FxYAML::buildScene()`. A visual application can pass it directly to the renderer; a headless app can step it itself.

```cpp
#include "Fx2D/Core.h"

int main() {
    auto scene = FxYAML::buildScene("Scene.yml");
    FxRylbRenderer renderer(scene, 60);
    renderer.run();
}
```

## Author entities in a useful order

For each entity, start with `pose`, then its physical behavior, then collision geometry. Keep names stable: joints, gameplay code, contacts, and queries all use them.

1. **Pose** is `[x, y, theta]`; YAML `theta` is in degrees.
2. **Physics** makes a body dynamic (`mass`) or immovable (`mass: 0` with `gravity_scale: 0`). Add damping, elasticity, friction, or CCD when needed.
3. **Collision** declares a circle, capsule, rectangle, convex polygon, edge, or chain.
4. **Visual** is optional and affects rendering only, not physics.

Static edges and chains make good terrain. Use circles, capsules, and convex polygons for dynamic bodies. See [Math and geometry](/guides/math) for how the same shapes work when you construct a scene in C++.

## Add mechanical relationships

The `joints:` section connects named entities. Revolute joints make hinges; prismatic joints make sliders. Put limits and motor settings in the joint declaration so the scene remains readable and editable.

```yaml
joints:
  wheel_hinge:
    type: revolute
    parent: chassis
    child: wheel
    anchor: [0, -0.5]
    angle_min: -0.6
    angle_max: 0.6
```

For motors and programmatic control, read [Joints and motors](/guides/joints). For direct XPBD constraints assembled in C++, use the [Constraints guide](/guides/constraints).

## Use the schema when you need an exact field

This guide is the authoring workflow. The [Scene YAML specification](/reference/scene-yaml) is the complete field-by-field reference, including every shape form, material value, visual option, joint parameter, and validation rule.
