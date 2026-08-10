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
   Build public scene/world APIs for:
   - ray casts
   - overlap queries
   - shape queries
   - sensors / trigger-only fixtures
   - buffered contact and sensor events exposed after each step

3. Add continuous collision support.
   Reduce tunneling for fast movers with:
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

## Why These Matter

- More shapes improve practical scene authoring and reduce the need for awkward polygon approximations.
- Query and event APIs make Fx2D more usable as an engine subsystem, not just a step-and-render loop.
- Continuous collision is one of the biggest gaps for fast-moving bodies.
- Better tests and solver regression coverage are key to making the engine more trustworthy as features grow.
