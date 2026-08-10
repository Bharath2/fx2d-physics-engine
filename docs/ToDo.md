# Fx2D Roadmap

This file tracks the next feature and robustness targets for Fx2D.

## Priority Targets

1. Add more collision shapes.
   Start with a few high-value shapes that unlock better level geometry and character collision:
   - ~~capsules~~ ✅ Done
   - ~~line segments / edges~~ ✅ Done (zero-radius capsule)
   - chain or polyline colliders
   - ~~rounded boxes or other rounded convex variants~~ ✅ Done (skin radius on any polygon)

   All shapes now share a unified `vertices[] + skin_radius` storage. `FxShape` recognises three
   types: `Circle` (0 vertices), `Capsule` (2 vertices), and `Polygon` (>=3 vertices); any of the
   latter two can carry a Minkowski-sum skin radius. YAML adds a `capsule:` key and an optional
   `radius:` modifier on `rectangle:` / `polygon:` for rounded variants.

   Edges are zero-skin capsules authored with `edge: [[x, y], [x, y]]`, intended for static level
   geometry. Edge-vs-polygon uses a dedicated line-reference query so a segment inside a polygon
   still reports contact; edge-vs-edge pairs and CCD on edges are intentionally skipped.

2. Add higher-level query and event systems.
   This is the gap between a physics demo and a game/RL substrate. The data already
   exists — `FxContact` carries up to 2 world-space contact points, the normal,
   penetration depth, applied impulses, and both entity pointers — but the contact
   list is a local variable inside `FxScene::step()` (`src/Scene.cpp`): computed,
   solved against, and thrown away every substep. The step callback fires after all
   that, so user code can never see a collision — "did the ball touch the goal?" is
   unanswerable today without re-doing collision math yourself.

   Build public scene/world APIs for:
   - buffered contact list exposed after each step (retain what `step()` already
     computes instead of discarding it)
   - begin/end contact events — the scene already keys contact pairs by a `uint64`
     id for warm-starting (`m_contact_cache`), so diffing pair ids across steps
     gives begin/end for little extra cost
   - sensors / trigger-only fixtures — a trigger is a contact that generates an
     event but no impulse; the plumbing is ~90% shared with the above
   - ray casts — lidar-style observations for RL, "what's under the mouse click"
     for a game
   - overlap queries
   - shape queries

   Suggested slices: (a) buffered contacts + begin/end events + triggers, which
   unblocks games and RL reward functions; then (b) ray/overlap/shape queries,
   which unblocks RL observations.

3. Make the collision pipeline faster and continuous.
   Two halves of the same pipeline: the broad phase decides *which pairs get
   looked at*; CCD is what actually prevents fast bodies passing through thin
   geometry. Division of labour matters — per-substep broad-phase queries never
   prevented tunneling (they sample AABBs at substep start; a fast body can
   cross a thin wall *within* one substep), so hoisting the query out of the
   substep loop costs no protection.

   **Broad-phase efficiency.** The tree itself is sound (SAH-guided dynamic
   AABB tree, fat boxes, dual-tree pair descent), but it is driven wastefully:
   `get_broad_phase_pairs()` runs per *substep* (`src/Scene.cpp`), so every
   frame pays N tree syncs + N full pair queries.
   - Query once per step over **full-step swept AABBs**
     (`combine(aabb, aabb + velocity * dt_full)`): any pair that can touch
     during any substep already overlaps in swept-box space at step start, so
     the once-per-step list is a superset of what per-substep queries find.
     Narrow phase still runs per substep on that list. The swept-box machinery
     already exists for CCD bodies in `Registry::get_broad_phase_pairs()` —
     apply it to all moving bodies with the full-step dt.
   - De-hash the hot path: store the tree node index on `FxEntity` instead of
     the `m_entity_node_map` / `m_entity_idx_map` lookups per entity/pair.
   - Reuse pair/contact buffers across calls instead of reallocating.
   - Trade: swept boxes admit a few more false-positive pairs (cheaply rejected
     by narrow phase) in exchange for one tree walk per step instead of N.
   - Edge case: a hard mid-step impact can redirect a fast body into geometry
     outside its swept path — mitigate with a small extra sweep margin, or let
     CCD bodies alone re-query per substep.

   **Continuous collision.** Reduce tunneling for fast movers with:
   - ~~speculative contacts~~ ✅ Done
     - `FxEntity::enable_ccd` flag (default `false`, zero overhead when off)
     - `FxSolver::speculative_contact_check()` generates a pre-contact (negative depth) when gap closes within the substep
     - Sleep filter in `Registry::get_broad_phase_pairs()` bypassed for CCD bodies
     - YAML `ccd:` key supported under the `physics:` block
   - time-of-impact style sweeps
   - fast-body or bullet-style handling for selected entities

4. ~~Harden broad-phase and collision robustness.~~ ✅ Done
   - Fixed fragile AABB sentinel check in `Registry::get_broad_phase_pairs()` using `FxAABB::is_valid()`
   - Static bodies (`inv_mass == 0`) no longer enter sleep state
   - Constrained entities are excluded from sleep-tick to prevent mid-joint drift
   - Restitution slop raised to `2e-2f` to suppress micro-bounce during stacking

5. ~~Expand test coverage and regression coverage.~~ ✅ Done
   Add more automated checks for:
   - ~~YAML scene loading~~ ✅ Done (`FxYAML::buildScene` on an inline scene in
     `tests/test_joints.cpp`; `FxYAML::buildShape` for capsule/rounded-rect/edge forms and
     error cases in `tests/test_capsule_collision.cpp` and `tests/test_collisions_edge.cpp`)
   - ~~joints and motor control~~ ✅ Done (`tests/test_joints.cpp`)
   - ~~collision manifolds and solver regressions~~ ✅ Done (`tests/test_capsule_collision.cpp`,
     `tests/test_collisions_edge.cpp`, `tests/test_resting_stability.cpp` — the last added with
     the resting-contact energy-leak fix)
   - ~~broad-phase updates and removal paths~~ ✅ Done (`tests/test_aabb_tree.cpp`)
   - ~~fast-moving body edge cases once CCD lands~~ ✅ Done (`tests/test_ccd.cpp`)

   The suite builds as one `Fx2DTests` binary under CTest; style and static
   analysis are separately gated in CI (`.github/workflows/lint.yml`,
   `scripts/lint.sh`).

6. Add more examples and docs around newer features.
   Prioritize:
   - ~~joint control examples for position, velocity, and effort~~ ✅ Done (`examples/joint_control_demo/`,
     covering revolute and prismatic motors in all three control modes)
   - ~~scene YAML examples that include joints~~ ✅ Done (`examples/joint_control_demo/Scene.yml`)
   - query/event examples — blocked on item 2 (query/event APIs not implemented yet)

7. Remove the float precision floor in position-level solving.
   Found while validating the joint example, where motors appeared dead at small timesteps.
   - ~~`FxAngleWrap` shifted by `+pi` before its `fmod`, which rounded away any angle below
     ~`1.2e-7` rad (half an ulp of float near pi) and returned exactly `0`. Substep rotations
     land in that range at small timesteps, so all angular motion silently froze.~~ ✅ Fixed —
     in-range angles now pass through untouched; covered by `tests/test_angle_precision.cpp`
   - Constraint corrections are still computed on `float` world coordinates, so a correction
     below one ulp of the coordinate magnitude (~`4.8e-7` at `x = 7`) is lost. A revolute joint
     with an anchor offset from the child's centre of mass depends on displacements at that
     scale, so at `dt = 1e-3` it loses most of its motor authority, and the error grows with
     distance from the origin. The same joint near the origin rotates ~5 orders of magnitude
     further per unit time. Fixing this means accumulating constraint corrections in double
     precision, or solving in body-relative coordinates rather than world coordinates.

8. Add renderer-level input hooks (keyboard/mouse).
   There is currently nothing: the renderer only has ImGui panel widgets, no key
   polling exists anywhere, and the examples drive motors programmatically.
   Practically, since raylib is linked anyway, `IsKeyDown(KEY_RIGHT)` can already
   be called inside `set_step_callback` to set joint targets — that works today
   for a quick game. The clean version is a proper renderer-level input hook, or
   an input-state struct passed to the step callback, so gameplay code does not
   have to reach into raylib directly (and headless scenes can be driven by the
   same interface).

## Why These Matter

- More shapes improve practical scene authoring and reduce the need for awkward polygon approximations.
- Query and event APIs make Fx2D more usable as an engine subsystem, not just a step-and-render loop.
- The collision pipeline work pays twice: hoisting the broad phase out of the substep loop removes the biggest per-frame waste, and continuous collision closes the biggest correctness gap for fast-moving bodies.
- Better tests and solver regression coverage are key to making the engine more trustworthy as features grow.
- Input hooks turn the renderer from a viewer into something a playable game can be built on, without gameplay code reaching into raylib.
