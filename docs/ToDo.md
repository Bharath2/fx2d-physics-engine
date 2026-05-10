# Fx2D Roadmap

This file tracks the next feature and robustness targets for Fx2D.

## Priority Targets

1. Add more collision shapes.
   Start with a few high-value shapes that unlock better level geometry and character collision:
   - capsules
   - line segments / edges
   - chain or polyline colliders
   - rounded boxes or other rounded convex variants

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

5. Expand test coverage and regression coverage.
   Add more automated checks for:
   - YAML scene loading
   - joints and motor control
   - collision manifolds and solver regressions
   - broad-phase updates and removal paths
   - fast-moving body edge cases once CCD lands

6. Add more examples and docs around newer features.
   Prioritize:
   - joint control examples for position, velocity, and effort
   - scene YAML examples that include joints
   - query/event examples once those APIs exist

## Why These Matter

- More shapes improve practical scene authoring and reduce the need for awkward polygon approximations.
- Query and event APIs make Fx2D more usable as an engine subsystem, not just a step-and-render loop.
- Continuous collision is one of the biggest gaps for fast-moving bodies.
- Better tests and solver regression coverage are key to making the engine more trustworthy as features grow.
