---
title: C++ API reference
sidebar: false
---

# C++ API reference

The complete API is generated from the public headers on every documentation deployment. It is deliberately separate from the guides: use the guides to understand a system, then use the generated reference for exact declarations and members.

<div class="api-map">
  <a href="/fx2d-physics-engine/cpp-api/index.html"><strong>Open the generated API →</strong><span>Classes, structs, enums, members, and the include graph.</span></a>
  <a href="/reference/scene-yaml"><strong>Scene YAML</strong><span>Declarative scene and entity configuration.</span></a>
  <a href="/reference/math"><strong>Math utilities</strong><span>Vectors, matrices, arrays, and geometric helpers.</span></a>
</div>

## Header map

| Header | Start here when you need… |
|---|---|
| `Fx2D/Core.h` | The standard rendering-enabled entry point. |
| `Fx2D/Scene.h` | Simulation stepping, entity lookup, events, queries, and groups. |
| `Fx2D/Entity.h` | Rigid-body state, material behavior, and shapes. |
| `Fx2D/Joints.h` | Revolute and prismatic constraints with motors. |
| `Fx2D/Geometry.h` | Shapes, AABBs, and ray-hit data. |
| `Fx2D/Math.h` | Engine vector, matrix, and `FxArray` types. |
| `Fx2D/Input.h` | Renderer-agnostic keyboard and mouse state. |

The generated API is based on `include/Fx2D/` and is refreshed automatically when public headers change.
