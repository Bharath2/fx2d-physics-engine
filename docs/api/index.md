---
title: C++ API
description: Choose the right Fx2D headers, link the static library, and navigate the generated symbol reference.
---

# C++ API

Start here when integrating Fx2D into another CMake project. The public API has two layers:

1. The **integration reference** on this site explains which header and library target to use.
2. The **generated symbol reference** documents every public class, struct, enum, and member from `include/Fx2D/`.

<div class="api-map">
  <a href="/fx2d-physics-engine/api/headers"><strong>Headers and entry points</strong><span>Choose `Core.h`, `Scene.h`, or a focused public header.</span></a>
  <a href="/fx2d-physics-engine/api/linking"><strong>Link Fx2Dlib</strong><span>Add Fx2D to a parent CMake project, headless or renderer-enabled.</span></a>
  <a href="/fx2d-physics-engine/api/headless"><strong>Headless simulation</strong><span>Create and step scenes with no raylib, ImGui, or window.</span></a>
  <a href="/fx2d-physics-engine/api/renderer"><strong>Visual applications</strong><span>Load YAML scenes and run the raylib renderer.</span></a>
</div>

## Choose an integration path

| You are building… | Include | Link target | Next |
|---|---|---|---|
| A visual app backed by a YAML scene | `Fx2D/Core.h` | `Fx2Dlib` | [Renderer integration](/api/renderer) |
| A headless YAML simulation | `Fx2D/Scene.h` + `Fx2D/YamlUtils.h` | `Fx2Dlib` built with `FX2D_HEADLESS=ON` | [Headless integration](/api/headless) |
| A programmatic physics world | `Fx2D/Scene.h`, `Fx2D/Entity.h` | `Fx2Dlib` built with `FX2D_HEADLESS=ON` | [Headers and entry points](/api/headers) |
| A specific engine feature | Its focused public header | `Fx2Dlib` | [Header map](/api/headers#public-header-map) |

## Generated symbol reference

Use the generated reference when you need signatures, fields, inheritance, or every member of a type. It is built from the repository headers during each Pages deployment.

<div class="api-map">
  <a href="/fx2d-physics-engine/cpp-api/index.html"><strong>All public symbols →</strong><span>Browse classes, structs, files, and include relationships.</span></a>
  <a href="/fx2d-physics-engine/cpp-api/classFxScene.html"><strong>FxScene →</strong><span>Stepping, entities, contacts, queries, groups, and reset.</span></a>
  <a href="/fx2d-physics-engine/cpp-api/classFxEntity.html"><strong>FxEntity →</strong><span>Body state, mass, geometry, forces, impulses, and sleep.</span></a>
  <a href="/fx2d-physics-engine/cpp-api/structFxShape.html"><strong>FxShape →</strong><span>Circles, capsules, polygons, chains, and geometry queries.</span></a>
</div>

> `Fx2Dlib` is the repository’s CMake target. Fx2D does not currently ship an installed `find_package(Fx2D)` package, so consume it with `add_subdirectory` or CMake `FetchContent` as shown in [linking Fx2D](/api/linking).
