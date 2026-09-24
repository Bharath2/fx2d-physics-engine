<div align="center">

# Fx2D

**A fast, deterministic 2D rigid-body physics engine in C++20.**
SAT collision detection · XPBD constraint solver · joints with motors · YAML scenes · headless mode for simulation, testing and reinforcement learning.

[![Docs](https://img.shields.io/badge/docs-bharath2.github.io%2Ffx2d--physics--engine-16b9c5?logo=readthedocs&logoColor=white)](https://bharath2.github.io/fx2d-physics-engine/)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/20)
[![License: BSD-3-Clause](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](./LICENSE)
[![GitHub stars](https://img.shields.io/github/stars/Bharath2/fx2d-physics-engine?style=social)](https://github.com/Bharath2/fx2d-physics-engine/stargazers)

[**Get started**](https://bharath2.github.io/fx2d-physics-engine/getting-started/install) ·
[**Demos**](https://bharath2.github.io/fx2d-physics-engine/demos) ·
[**API reference**](https://bharath2.github.io/fx2d-physics-engine/api/) ·
[**Try it in your browser**](https://bharath2.github.io/fx2d-physics-engine/playground) 

<img src="./examples/angry_boxes/play.gif" alt="Angry Boxes demo: a mouse-driven slingshot launches a ball into a tower of boxes simulated by the Fx2D physics engine" width="720" />

<sub>`examples/angry_boxes` — drag the ball, release, topple the tower. Mouse input, contact impulses and a draw overlay in ~150 lines.</sub>

</div>

---

## Why Fx2D?

- **Physics you can read.** One explicit pipeline: a dynamic AABB tree proposes pairs, a skin-aware SAT narrow phase produces contacts across *every* shape type, and a substepped XPBD solver resolves contacts, friction and joints. The [solver](https://bharath2.github.io/fx2d-physics-engine/concepts/xpbd) and [collision](https://bharath2.github.io/fx2d-physics-engine/concepts/collisions) docs derive the equations the code implements.
- **Built for simulation, not just games.** A renderer-free build steps thousands of times per second with no window, no GPU and no raylib. Inject keyboard and mouse state programmatically, read contacts, cast rays, and batch rollouts for RL or data collection.
- **Deterministic and tested.** Fixed timestep, fixed solver ordering, no threads by default. An adversarial test suite covers tall stacks, pyramids, 10:1 and 100:1 mass ratios, paper-thin slivers, a Newton's cradle, spinning bodies and kinematic platforms, with thresholds that were measured rather than guessed.
- **Declarative scenes.** Describe worlds, textures, joints and physics parameters in YAML, then load them from C++ in one line. Reset a scene to its authored state at any time.
- **Small and embeddable.** About 7k lines of C++20, a single static library, BSD-3-Clause. Depends only on Eigen and yaml-cpp in headless mode.

## Features

| Area | What you get |
|---|---|
| **Shapes** | Circles, capsules, edges, chains (open polylines for terrain), convex polygons, and rounded rectangles/polygons via a skin radius, all in one `vertices[] + skin_radius` representation |
| **Broad phase** | SAH-guided dynamic AABB tree with fat boxes and dual-tree pair descent |
| **Narrow phase** | Separating Axis Theorem with skin-aware contact generation, clipping manifolds, one-sided chain contacts |
| **Continuous collision** | Opt-in speculative contacts (`ccd: true`) to curb tunneling for fast bodies |
| **Solver** | Substepped XPBD with compliance, warm starting, restitution, and Coulomb static/dynamic friction. Default 14 substeps × 4 velocity passes, chosen by measurement |
| **Joints & motors** | Revolute and prismatic joints with position, velocity and effort control modes and PID tuning |
| **Mouse joint** | A damped spring from the cursor to a grabbed body, tuned by frequency and damping ratio so it scales with mass; `mouse_drag: true` in a scene gives click-and-drag for free |
| **Queries** | Ray casts, overlap (circle/box/point/shape) and point picking that share the simulation's own narrow phase |
| **Contacts & events** | Buffered contacts each step, begin/end contact events, trigger-only sensor entities |
| **Entity groups** | Named sets you bulk enable/delete/reset, plus one-integer intra-group collision filtering |
| **Input** | Renderer-agnostic keyboard and mouse with world-space cursor; identical API when injected headlessly |
| **Sleeping** | Resting bodies stop consuming solver time until disturbed |
| **Scenes** | YAML scene description for entities, textures, joints and solver parameters, with reset callbacks |
| **Headless** | Build the physics core with no renderer for tests, CI, batch simulation and RL |
| **Math** | NumPy-style `FxArray`, vector/matrix helpers and geometry utilities in `Fx2D/Math.h` |
| **Rendering** | Lightweight cross-platform viewer on raylib with a Dear ImGui inspector and a draw-callback hook for overlays |

## Quick start

```bash
git clone https://github.com/Bharath2/fx2d-physics-engine.git
cd fx2d-physics-engine
./scripts/build_headless.sh          # needs only Eigen3, yaml-cpp and TBB
./build-headless/fx2d_tests          # run the test suite
./build-headless/truck_headless      # step a scene with no window
```

Then write a scene and step it:

```yaml
# Scene.yml
scene:
  size: [16, 9]
  gravity: [0, -9.81]

entities:
  floor:
    pose: [8, 0.5, 0]
    physics: { mass: 0, gravity_scale: 0 }
    collision: { geometry: { rectangle: [16, 1] } }

  ball:
    pose: [8, 7, 0]
    physics: { mass: 1.0, elasticity: 0.5 }
    collision: { geometry: { circle: 0.25 } }
```

```cpp
#include "Fx2D/Scene.h"
#include "Fx2D/YamlUtils.h"

int main() {
    auto scene = FxYAML::buildScene("Scene.yml");
    auto ball  = scene.get_entity("ball");
    for (int i = 0; i < 600; ++i) {
        scene.step(1.0 / 60.0);          // fixed timestep, deterministic
        std::cout << ball->pose << '\n'; // x, y, theta
    }
}
```

Want a window? Include `Fx2D/Core.h` instead and hand the scene to the renderer:

```cpp
#include "Fx2D/Core.h"

int main() {
    auto scene = FxYAML::buildScene("Scene.yml");
    FxRylbRenderer renderer(scene, 60);
    renderer.run();
}
```

The [installation guide](https://bharath2.github.io/fx2d-physics-engine/getting-started/install) covers the visual build (raylib, Dear ImGui, rlImGui) on Linux, macOS, Windows and MSYS2.

## Examples

Every example is a plain C++ file plus a `Scene.yml` under [`examples/`](./examples/). Build them with `-DFX2D_BUILD_EXAMPLES=ON` and run from the repository root so asset paths resolve.

<table>
  <tr>
    <td align="center" width="50%">
      <a href="./examples/playground/"><img src="./examples/playground/play.gif" alt="Playground demo" /></a><br />
      <b><a href="./examples/playground/">Playground</a></b><br />
      <sub>Mouse joint drag, spawn shapes, seesaw, chain ramp. Also runs in the browser</sub>
    </td>
    <td align="center" width="50%">
      <a href="./examples/angry_boxes/"><img src="./examples/angry_boxes/play.gif" alt="Angry Boxes slingshot demo" /></a><br />
      <b><a href="./examples/angry_boxes/">Angry boxes</a></b><br />
      <sub>Mouse slingshot, trajectory preview, contact impulses, reset</sub>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <a href="./examples/truck/"><img src="./examples/truck/play.gif" alt="Truck suspension demo" /></a><br />
      <b><a href="./examples/truck/">Truck</a></b><br />
      <sub>Wheels, suspension joints, textured bodies</sub>
    </td>
    <td align="center" width="50%">
      <a href="./examples/stacked_boxes/"><img src="./examples/stacked_boxes/play.gif" alt="Stacked boxes demo" /></a><br />
      <b><a href="./examples/stacked_boxes/">Stacked boxes</a></b><br />
      <sub>Resting stability, friction, textured shapes</sub>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <a href="./examples/joint_control_demo/"><img src="./examples/joint_control_demo/play.gif" alt="Joint motor control demo" /></a><br />
      <b><a href="./examples/joint_control_demo/">Joint control</a></b><br />
      <sub>Revolute and prismatic motors: position, velocity, effort modes</sub>
    </td>
    <td align="center" width="50%">
      <a href="./examples/chain_terrain/"><img src="./examples/chain_terrain/play.gif" alt="Chain terrain demo" /></a><br />
      <b><a href="./examples/chain_terrain/">Chain terrain</a></b><br />
      <sub>Polyline terrain, click-to-spawn, one-sided chain contacts</sub>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <a href="./examples/bucket_fill/"><img src="./examples/bucket_fill/play.gif" alt="Bucket fill demo" /></a><br />
      <b><a href="./examples/bucket_fill/">Bucket fill</a></b><br />
      <sub>Hundreds of bodies piling into a container, sleeping, broad phase under load</sub>
    </td>
    <td width="50%"></td>
  </tr>
</table>

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DFX2D_BUILD_EXAMPLES=ON
cmake --build build -j
./build/example_playground       # also: example_angry_boxes, example_truck, example_stacked_boxes,
                                 # example_joint_control, example_chain_terrain, example_bucket_fill
```

## Headless simulation and reinforcement learning

The physics core has no dependency on the renderer. Build with `-DFX2D_HEADLESS=ON` (or `./scripts/build_headless.sh`) and you get the same engine with no raylib, no Dear ImGui and no window, which is what CI runs.

```cpp
FxScene scene = FxYAML::buildScene("examples/angry_boxes/Scene.yml");

// Drive the scene the way a player would, without a cursor.
scene.input().set_mouse_position(world_xy, screen_xy);
scene.input().set_mouse_button(FxMouseButton::Left, true);
scene.step(dt);

// Observe: contacts, impulses, ray casts, overlaps.
for (const auto& c : scene.contacts()) { /* c.entity1, c.normal, c.jn_accumulated */ }
FxRayHit hit;
if (scene.raycast(origin, direction, max_distance, hit)) { /* hit.entity, hit.point */ }
scene.reset();  // back to the authored scene, groups included
```

Because stepping is deterministic and single-threaded, many independent scenes can run in parallel across processes or threads for batched rollouts. See the [headless guide](https://bharath2.github.io/fx2d-physics-engine/guides/headless), [input](https://bharath2.github.io/fx2d-physics-engine/guides/input), [queries](https://bharath2.github.io/fx2d-physics-engine/guides/queries) and [contacts and sensors](https://bharath2.github.io/fx2d-physics-engine/guides/events).

## How it works

```
        step(dt)
          │
          └─ for each substep (default 14):
                ├─ joint motor controls, then integrate (gravity, forces, damping)
                ├─ broad phase ── dynamic AABB tree → candidate pairs
                ├─ narrow phase ── skin-aware SAT → contact manifolds (cached, warm-started)
                ├─ XPBD position solve ── penetration, joints, compliance
                ├─ derive velocities from the position change
                └─ velocity passes (default 4) ── restitution and Coulomb friction
```

Read the derivations in [XPBD solver](./docs/concepts/xpbd.md) and [collision pipeline](./docs/concepts/collisions.md), and the reasoning behind the defaults in the [roadmap](./docs/roadmap.md), which records every measurement that shaped the engine.

## Dependencies

| Dependency | Needed for | Version |
|---|---|---|
| [CMake](https://cmake.org/) | Build | 3.16+ |
| [Eigen3](https://eigen.tuxfamily.org/) | Math | 3.3+ |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | Scene files | any recent |
| [raylib](https://www.raylib.com/) | Renderer only | 4.5+ |
| [Dear ImGui](https://github.com/ocornut/imgui) + [rlImGui](https://github.com/raylib-extras/rlImGui) | Renderer only | ImGui 1.92 |

The repository keeps `lib/imgui` and `lib/rlImGui` as empty placeholders; clone the upstream sources into them before a visual build. Headless builds need none of the renderer dependencies.

### Build

```bash
# CMake
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

# or the helper
./fxmake            # Release
./fxmake debug      # Debug
./fxmake rebuild    # clean + rebuild
```

## Documentation

| Doc | Description |
|---|---|
| [Documentation site](https://bharath2.github.io/fx2d-physics-engine/) | Guided introduction, runnable demos, and full API reference |
| [Scene YAML](./docs/reference/scene-yaml.md) | Scene blocks, entities, geometry types, joints and physics fields |
| [XPBD solver](./docs/concepts/xpbd.md) | Per-substep pipeline, constraint kernel equations, and constraint types |
| [Collision pipeline](./docs/concepts/collisions.md) | SAT narrow phase, penetration correction, restitution, and friction |
| [Guides](./docs/guides/index.md) | Headless simulation, input, queries, contacts, groups, renderer, and joints |
| [Math utilities](./docs/reference/math.md) | `FxArray`, vector/matrix types, and helper functions |
| [Roadmap](./docs/roadmap.md) | What is next, with the measurements behind each decision |

## Roadmap highlights

Delivered most recently: the mouse joint and the browser playground.

- **SIMD solver**: structure-of-arrays gather/scatter inside `step()`, then a graph-colored 8-wide velocity solve ([plan](./docs/roadmap/simd.md)).
- **Ropes and bridges**: distance joints and a dynamic chain mode.
- **Time-of-impact CCD** so fast bodies never tunnel through chains and edges.
- **More joints**: weld, wheel, pulley, gear.

## Contributing

Contributions are welcome. See [CONTRIBUTING](./docs/contributing.md) for the workflow, the lint gate (`./scripts/lint.sh`) and how to run the test suite. Open roadmap items each record the motivation, the relevant code paths and a suggested approach, so they make good first issues.

If Fx2D is useful to you, a ⭐ on the repository helps others find it.

## Citation

```bibtex
@software{fx2d,
  author  = {Irigireddy, Bharath Chandra},
  title   = {Fx2D: A 2D rigid-body physics engine in C++20},
  year    = {2025},
  url     = {https://github.com/Bharath2/fx2d-physics-engine}
}
```

## License

[BSD-3-Clause](./LICENSE)
