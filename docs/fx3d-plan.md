# Fx3D Plan — a 3D engine from the Fx2D codebase

How much of Fx2D carries into a 3D engine, what has to be rebuilt, and the
strategy that makes the project tractable.

The direction of travel is natural: the XPBD literature is **3D-first** —
Müller et al., *Detailed Rigid Body Simulation with XPBD*, gives the 3D
constraint recipes directly. Fx2D is the specialization, so going to 3D is a
return to the source material, not a leap into the unknown.

## Reuse map, module by module

| Module | Reuse | What changes |
|---|---|---|
| `FxArray`, math utilities | ~95% | Nothing structural. |
| `AABBTree.h` | ~90% | `FxAABB` gains z; the SAH cost metric switches perimeter → surface area. The algorithm — SAH-guided insertion, fat boxes, dual-tree pair descent — is dimension-blind. |
| `Registry.h` (entities, pair filter, sleep) | ~90% | Only the AABB type. |
| Scene step loop (substeps, warm-start cache, sleep logic) | ~70% | Structure identical; math swapped. |
| XPBD constraint kernels (`Constraints.cpp`) | ~50% | The compliance/Lagrange scheme carries verbatim; the generalized inverse mass becomes quaternion + world-space inertia tensor (`R·I·Rᵀ`). Eigen already provides `Quaternionf`. |
| Entity | ~50% | `pose {x, y, θ}` → position + quaternion; scalar inertia → 3×3 tensor. |
| Joints | ~40% | Revolute → hinge (two angular locks + one driven axis), prismatic similar; the paper has exact recipes, and the motor/PID layer carries. |
| YAML loader | ~80% | Schema grows z and an orientation representation. |
| CCD (speculative contacts) | concept carries | Same gap-closing idea in 3D; swept AABB inflation is identical. |
| Renderer | ~30% | raylib supports 3D (`Camera3D`, meshes) — a real rewrite, but the cheapest possible one; the ImGui panel carries. |
| **Collision detection (`Collisions.cpp`)** | **~15%** | **This is the project.** 2D SAT + edge clipping → 3D needs GJK/EPA (or 3D SAT for boxes with edge-edge cross axes) plus 3D contact-manifold generation (reference-face clipping). Half the total effort, and where all the numerical robustness pain lives. |

## Strategy: shape-limited Fx3D, not general Fx3D

Restrict v1 to **sphere, capsule, box, plane** and every collision routine has
a closed form or a simple special case:

- sphere vs anything is trivial;
- capsule–capsule is segment–segment distance, which the 2D capsule work
  already established;
- box–box goes through SAT with edge-edge cross-product axes;
- plane vs anything is a half-space test.

GJK/EPA and general convex hulls are skipped entirely in v1 — they are the
long tail, not the entry fee.

The payoff: **sphere + capsule + box + hinge motors is exactly the MuJoCo body
plan** — ragdolls, quadrupeds, arms. Combined with the headless core, YAML
scene authoring, and the RL-environment niche (see the roadmap discussion of
queries/events), shape-limited Fx3D is not a lesser 3D engine; it is a
*MuJoCo-lite for RL*. That is a far more valuable target than generic 3D
physics, where the competition is Jolt and PhysX.

## Sequencing

1. **3D math core** — quaternion pose, inertia tensors, `FxAABB3`. Small but
   must be done carefully; everything sits on it.
2. **Port the XPBD kernels + hinge/slider joints** from the paper's recipes.
3. **Sphere/capsule/box/plane collision + manifolds** — the long pole.
4. **3D AABB tree + registry** — near-mechanical given the reuse map.
5. **raylib 3D viewer** — `Camera3D`, mesh drawing, the existing ImGui panel.
6. **Port the stability test suite** — plus the adversarial scenes from
   roadmap item 8 (tall stacks, mass ratios, loaded chains), in 3D from day
   one. Robustness is bought with scenes, not architecture.

## Do these 2D items first

Roadmap items **2** (queries/events), **3** (broad-phase hoist), and **7**
(parallel narrow phase / island solve) port to 3D essentially for free if they
are built before the fork — and are pure rework if built after. Land them in
Fx2D first.

## Repo mechanics

Fork to a separate `fx3d` repo rather than templating the core on dimension.
Shared-source 2D/3D engines are a maintenance tarpit; shared *architecture* is
the right reuse. The fork keeps Fx2D releasable and lets the 3D collision work
proceed without destabilizing the 2D engine.
