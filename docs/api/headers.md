---
title: Headers and entry points
---

# Headers and entry points

Fx2D is a C++20 static library. Include the narrowest public header that fits the job; every public header lives under `include/Fx2D/`.

## Pick the top-level include

### Visual YAML application

```cpp
#include "Fx2D/Core.h"
```

`Core.h` is the convenience include for a complete visual application. It brings in the scene, YAML builder, and raylib/ImGui renderer. It therefore requires a non-headless build.

### Headless YAML application

```cpp
#include "Fx2D/Scene.h"
#include "Fx2D/YamlUtils.h"
```

This is the right split for testing, simulation, or data collection. It can load a `Scene.yml`, but does not include raylib, Dear ImGui, or rlImGui.

### Programmatic simulation

```cpp
#include "Fx2D/Scene.h"
#include "Fx2D/Entity.h"
```

Add `Geometry.h` when constructing shapes explicitly, and `Joints.h` when creating joints in C++.

## Public header map

| Header | Provides | Use it for |
|---|---|---|
| [`Core.h`](https://github.com/Bharath2/fx2d-physics-engine/blob/main/include/Fx2D/Core.h) | Scene + YAML + renderer | The all-in-one visual entry point. |
| [`Scene.h`](https://github.com/Bharath2/fx2d-physics-engine/blob/main/include/Fx2D/Scene.h) | `FxScene`, groups, contact events, queries | Owning and stepping a physics world. |
| [`Entity.h`](https://github.com/Bharath2/fx2d-physics-engine/blob/main/include/Fx2D/Entity.h) | `FxEntity`, `FxVisualShape` | Bodies, material properties, forces, and impulses. |
| [`Geometry.h`](https://github.com/Bharath2/fx2d-physics-engine/blob/main/include/Fx2D/Geometry.h) | `FxShape`, `FxAABB`, `FxRayHit` | Circles, capsules, polygons, chains, and query data. |
| [`Joints.h`](https://github.com/Bharath2/fx2d-physics-engine/blob/main/include/Fx2D/Joints.h) | `FxJoint`, revolute/prismatic joints | Constraints and motor control. |
| [`YamlUtils.h`](https://github.com/Bharath2/fx2d-physics-engine/blob/main/include/Fx2D/YamlUtils.h) | `FxYAML::buildScene` and builders | Loading YAML scenes or pieces of a scene. |
| [`Renderer.h`](https://github.com/Bharath2/fx2d-physics-engine/blob/main/include/Fx2D/Renderer.h) | `FxRylbRenderer` | Window, camera, input polling, and draw callbacks. |
| [`Input.h`](https://github.com/Bharath2/fx2d-physics-engine/blob/main/include/Fx2D/Input.h) | `FxInput`, `FxKey`, `FxMouseButton` | Renderer-agnostic input and headless input injection. |
| [`Math.h`](https://github.com/Bharath2/fx2d-physics-engine/blob/main/include/Fx2D/Math.h) | `FxVec*`, `FxMat*`, `FxArray` | Engine math types and shape construction data. |

## Direct type routes

The generated API opens directly to these core types:

- [FxScene](/fx2d-physics-engine/cpp-api/classFxScene.html) — simulation, callbacks, contacts, queries, and groups.
- [FxEntity](/fx2d-physics-engine/cpp-api/classFxEntity.html) — rigid body state and forces.
- [FxShape](/fx2d-physics-engine/cpp-api/structFxShape.html) — the unified collision shape.
- [FxRylbRenderer](/fx2d-physics-engine/cpp-api/classFxRylbRenderer.html) — visual loop and transforms.
- [FxRevoluteJoint](/fx2d-physics-engine/cpp-api/classFxRevoluteJoint.html) and [FxPrismaticJoint](/fx2d-physics-engine/cpp-api/classFxPrismaticJoint.html) — motorized joints.

These links stay on the symbol route; they do not redirect to the docs landing page.
