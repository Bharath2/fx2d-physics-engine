# Fx2D Roadmap

This file tracks the next feature and robustness targets for Fx2D. Delivered
targets are removed; see git history for what already landed (shapes unified
under `vertices[] + skin_radius`, capsules/edges/rounded variants, speculative-
contact CCD, broad-phase hardening, the test suite, joint-control examples,
the `FxAngleWrap` precision fix, contacts/events/sensors, and the YAML inertia
ordering fix).

## Priority Targets

1. Add chain / polyline colliders.
   The last shape from the original shape list. Shapes share a unified
   `vertices[] + skin_radius` storage — `FxShape` recognises `Circle`
   (0 vertices), `Capsule` (2 vertices), and `Polygon` (>=3 vertices) — and
   edges already exist as zero-skin capsules (`edge: [[x, y], [x, y]]`) with a
   dedicated line-reference query. A chain collider is the natural composition:
   a sequence of edge segments authored as one entity, for static level
   geometry that a polygon approximates awkwardly. Edge-vs-edge pairs and CCD
   on edges are intentionally skipped today; chains inherit that.

2. Add higher-level spatial query APIs.
   Slice (a) — buffered contacts, begin/end contact events, and sensors — has
   landed. `FxScene::contacts()`, `begin_contact_events()`, `end_contact_events()`,
   and `FxEntity::is_sensor` (YAML `sensor:`) are documented in
   [contacts_and_events.md](contacts_and_events.md) and covered by
   `tests/test_contact_events.cpp`. Reward functions and "did the ball reach the
   goal?" are answerable now.

   Remaining is slice (b), the observation half, which nothing else is blocked on:
   - ray casts — lidar-style observations for RL, "what's under the mouse click"
     for a game
   - overlap queries — which entities intersect this box/circle
   - shape queries — sweep an arbitrary shape and report what it hits

   All three want the same entry point the broad phase already has: the dynamic
   AABB tree in `FxEntityRegistry` (`include/Fx2D/Registry.h`) supports descent
   from a query volume, so a ray cast is a tree walk plus a narrow-phase segment
   test per candidate. The per-shape segment tests partly exist already: edges use
   a dedicated line-reference query in `src/Collisions.cpp`.

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

   **Continuous collision.** Speculative contacts are done
   (`FxEntity::enable_ccd`, `FxSolver::speculative_contact_check()`, YAML
   `ccd:` key). Remaining, to reduce tunneling further for fast movers:
   - time-of-impact style sweeps
   - fast-body or bullet-style handling for selected entities

4. Add more examples and docs around newer features.
   - a worked contact-event example — a sensor goal region plus impact-strength
     readout, now that item 2 slice (a) makes it expressible
   - ray-cast / overlap examples — still blocked on item 2 slice (b)

5. Remove the float precision floor in position-level solving.
   Found while validating the joint example, where motors appeared dead at small
   timesteps; the `FxAngleWrap` half is fixed and regression-covered
   (`tests/test_angle_precision.cpp`). The remaining half:
   - Constraint corrections are still computed on `float` world coordinates, so a correction
     below one ulp of the coordinate magnitude (~`4.8e-7` at `x = 7`) is lost. A revolute joint
     with an anchor offset from the child's centre of mass depends on displacements at that
     scale, so at `dt = 1e-3` it loses most of its motor authority, and the error grows with
     distance from the origin. The same joint near the origin rotates ~5 orders of magnitude
     further per unit time. Fixing this means accumulating constraint corrections in double
     precision, or solving in body-relative coordinates rather than world coordinates.

6. Add renderer-level input hooks (keyboard/mouse).
   There is currently nothing: the renderer only has ImGui panel widgets, no key
   polling exists anywhere, and the examples drive motors programmatically.
   Practically, since raylib is linked anyway, `IsKeyDown(KEY_RIGHT)` can already
   be called inside `set_step_callback` to set joint targets — that works today
   for a quick game. The clean version is a proper renderer-level input hook, or
   an input-state struct passed to the step callback, so gameplay code does not
   have to reach into raylib directly (and headless scenes can be driven by the
   same interface).

7. Parallelize the narrow phase and the island solve.
   The engine is single-threaded today. Leaf-level SIMD hygiene already exists —
   `-O3 -march=native` under `FX2D_NATIVE`, 32-byte-aligned `FxArray` with
   `__restrict` loops that auto-vectorize — but the solver hot path is AoS
   `FxVec2f`/`FxVec3f` math through `shared_ptr` entities, which neither
   vectorizes wide nor threads. Ranked by value/effort:
   - **Narrow phase (easy, biggest win).** Each pair's `collision_check` in
     `FxScene::step()` (`src/Scene.cpp`) is independent pure geometry against
     const entity state. Parallel-for over `broad_phase_pairs` with per-thread
     contact buffers, concatenated **in pair order** (never completion order —
     that keeps determinism). Hoist the shared-state bits currently inside the
     loop — `wake_if_disturbed`, `active_keys.insert`, warm-start cache lookup —
     into a cheap serial pass over the produced contacts. OpenMP is the least
     invasive mechanism; a small thread pool avoids the dependency.
   - **Entity integration.** The `entity->step()` and velocity-derivation loops
     are trivially parallel, but only pay off in the thousands-of-bodies range.
   - **Island-based parallel solve (structurally correct big one).** The XPBD
     position solve and velocity sweeps are Gauss-Seidel: sequential within a
     group of touching bodies, but bodies only couple through contacts/joints —
     so disconnected islands solve independently. Union-find over contact pairs
     + joints → one task per island; also unlocks per-island sleeping. Payoff
     scales with scene fragmentation (many separate stacks ≈ linear speedup;
     one giant pile ≈ none).
   - **Graph coloring within an island** (Jacobi-style, how XPBD runs on GPUs)
     — only worth it for huge single islands; changes convergence slightly.
     Skip until proven needed.
   - Discipline throughout: fixed reduction order for float accumulation and no
     parallel writes to `m_contact_cache` (write-back stays serial) — otherwise
     the sim goes nondeterministic, which would hurt the RL story.
   - Out of scope for now: SoA/SIMD batch solving of contacts (Box2D v3 style)
     — a large refactor that only matters after the above land.

8. Prove the solver against adversarial scenes.
   The solver's *design* is contemporary — 11 substeps × position solve + 8
   velocity passes with warm-starting, normal-flip detection on the friction
   basis, a restitution speed floor, and accumulated normal/tangent impulses is
   the same substepping-with-soft-constraints generation Box2D v3 moved to.
   What is missing is not architecture but *proof*: the stability suite covers
   no-jitter, no-creep, a three-box equal-mass stack, bounce energy, friction
   stop, and bounded penetration — all correct things to test, and all easy
   mode. The proven envelope today is small scenes at benign mass ratios.
   Add the scenes mature engines burned years on, as regression tests, and fix
   what falls out:
   - **Tall stacks** — 10/20/30 boxes and pyramid stacks. Substepping should
     handle these well; "should" is untested.
   - **High mass ratios** — a 100:1 anvil on a feather box; a heavy body atop
     a light stack. The classic solver killer; nothing in the suite goes
     beyond ~equal masses.
   - **Chains under tension** — long revolute chains (rope, bridge) with a
     weight at the end; joint stretch and motor authority under load. Item 5's
     float-precision finding (motor authority dying with distance from origin)
     was discovered *by accident* in this class — direct evidence it holds
     more landing mines.
   - **The rest of the classics** — degenerate geometry (thin slivers),
     restitution chains (Newton's cradle), fast-spinning bodies, kinematic
     platforms.
   Item 5's remaining half is the first known bug this suite would pin down;
   expect it to surface others. Tractable, just grinding.

## Why These Matter

- Chain colliders finish practical scene authoring for static level geometry.
- Query APIs make Fx2D more usable as an engine subsystem, not just a step-and-render loop. Contacts and events (slice a) covered the reward/game-logic half; ray and overlap queries cover the observation half.
- The collision pipeline work pays twice: hoisting the broad phase out of the substep loop removes the biggest per-frame waste, and continuous collision closes the biggest correctness gap for fast-moving bodies.
- Input hooks turn the renderer from a viewer into something a playable game can be built on, without gameplay code reaching into raylib.
- Parallelism raises the body-count ceiling without touching solver behavior — narrow phase first because it is embarrassingly parallel, islands second because they preserve Gauss-Seidel convergence.
- Adversarial scenes are how solver robustness is actually bought — mature engines earned their trust against tall stacks, mass ratios, and loaded chains, not through architecture; each scene added is envelope the solver provably owns.
