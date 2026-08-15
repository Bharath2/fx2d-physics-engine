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

8. Fix what the adversarial scenes found.
   The scenes themselves have landed (`tests/test_adversarial.cpp`): tall stacks,
   pyramids, mass ratios, thin slivers, a restitution chain, spinning bodies, and
   a kinematic platform. They pin the envelope the solver provably owns, and the
   thresholds in them are measured rather than aspirational. What they proved:

   **Works.** Columns up to 15 boxes and a 5-wide pyramid hold at the default 11
   substeps. A 20-box column holds at 22. Raising substeps monotonically reduces
   sink, so the knob users reach for genuinely works. 10:1 mass ratios are
   near-exact and 100:1 holds to within 0.073 of a unit box height. Thin slivers
   (down to 0.02 thick) rest without jitter or tilt. Newton's cradle transfers
   momentum correctly, leaving the middle balls in place. A box rides a
   velocity-driven kinematic platform.

   Remaining, in the order the evidence justifies:

   - **Spin injects energy into the contact solve.** A unit box spun in place on
     the ground may legitimately rise to corner-pivot height (centre at
     `1.5 + sqrt(2)/2 = 2.207`). Up to 15 rad/s it reaches exactly that and no
     more. From 20 rad/s it climbs past it — 2.31 at 20, 2.67 at 30, 3.74 at 50,
     8.89 at 100 — so the solver is manufacturing height out of rotation. At
     100 rad/s the box launches to y ≈ 8.9 while angular velocity drops 100 → 28.6:
     roughly 70% of the spin is converted into a 6.7 m/s upward throw. It then
     falls back at ~13 m/s and tunnels through the 1-unit-thick floor. This is the
     clearest correctness bug the suite found, and the tunneling is only its
     symptom — fixing the energy gain removes the fall-through with it. Note CCD
     makes no difference, because `speculative_contact_check()` computes closing
     speed from linear velocity alone and ignores angular velocity entirely.
   - **High mass ratios creep without converging.** A 100 kg box dropped on a
     stack of five 1 kg boxes sinks continuously rather than settling: over 120
     steps the whole stack descends steadily and never reaches equilibrium, ending
     collapsed into the ground. A static 1000:1 pair is similar, burying the light
     box half its height. The 100:1 resting pair is fine, so the failure is about
     accumulated load through a chain of contacts, not the ratio alone.
   - **Chains under tension are still untested.** The one class from the original
     list not yet covered — long revolute chains (rope, bridge) with a weight at
     the end, testing joint stretch and motor authority under load. Item 5's
     float-precision finding was discovered *by accident* in this class, which is
     direct evidence it holds more. Worth resolving the duplicate-constraint
     warning (`FxNamedRegistry: Item 'base_link_Anchor' already exists.`, printed
     by the joint tests) first, since chain tests depend on joint constraints
     registering the way the author expects.

   Note the suite is marked slow and skipped when `FX2D_SKIP_SLOW_TESTS=1`, which
   CI sets for its Debug/sanitizer job only. Release runs it on every push.

## Why These Matter

- Chain colliders finish practical scene authoring for static level geometry.
- Query APIs make Fx2D more usable as an engine subsystem, not just a step-and-render loop. Contacts and events (slice a) covered the reward/game-logic half; ray and overlap queries cover the observation half.
- The collision pipeline work pays twice: hoisting the broad phase out of the substep loop removes the biggest per-frame waste, and continuous collision closes the biggest correctness gap for fast-moving bodies.
- Input hooks turn the renderer from a viewer into something a playable game can be built on, without gameplay code reaching into raylib.
- Parallelism raises the body-count ceiling without touching solver behavior — narrow phase first because it is embarrassingly parallel, islands second because they preserve Gauss-Seidel convergence.
- Adversarial scenes are how solver robustness is actually bought — mature engines earned their trust against tall stacks, mass ratios, and loaded chains, not through architecture; each scene added is envelope the solver provably owns. The scenes now exist and have already located a real energy-injection bug in the contact solve.
