---
title: Guides
---

# Build with Fx2D

These guides are organized around the parts of an application you will actually build: author a scene, run it with or without a window, then add interaction and game logic.

- [Scene authoring](/reference/scene-yaml) — create entities, shapes, physical properties, and joints.
- [Math and geometry](/guides/math) — use vectors, poses, transforms, and the unified shape model safely.
- [Collision setup](/guides/collisions) — configure colliders, materials, sensors, filtering, and CCD.
- [Constraints](/guides/constraints) — assemble direct XPBD constraints or choose a motorized joint.
- [Headless simulation](/guides/headless) — run the same physics core without a renderer.
- [Renderer](/guides/renderer) — own the window, camera, frame timing, and draw callbacks.
- [Input](/guides/input) — consume keyboard and mouse state in visual or headless applications.
- [Joints and motors](/guides/joints) — use revolute and prismatic control modes.
- [Queries](/guides/queries) — ray casts, overlaps, and point queries for gameplay and tools.
- [Contacts and sensors](/guides/events) — respond to collisions and trigger volumes.
- [Entity groups](/guides/entity-groups) — operate on named sets of bodies safely.

For the implementation details behind these building blocks, see the [collision pipeline](/concepts/collisions) and the [XPBD solver](/concepts/xpbd).
