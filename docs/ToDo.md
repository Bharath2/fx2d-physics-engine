# Fx2D Roadmap

This file tracks the next feature and robustness targets for Fx2D. Delivered
targets are removed; see git history for what already landed (shapes unified
under `vertices[] + skin_radius`, capsules/edges/rounded variants, speculative-
contact CCD, broad-phase hardening, the test suite, joint-control examples,
the `FxAngleWrap` precision fix, contacts/events/sensors, the YAML inertia
ordering fix, the adversarial scene suite, and keyboard/mouse input).

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

2. Spatial query APIs — delivered.
   Both slices have landed. Slice (a): buffered contacts, begin/end contact events and sensors
   (`FxScene::contacts()`, `begin_contact_events()`, `end_contact_events()`,
   `FxEntity::is_sensor`), documented in [contacts_and_events.md](contacts_and_events.md).
   Slice (b): ray casts, overlap and point queries (`raycast()`, `raycast_all()`,
   `overlap_circle/box/point/shape()`, `entity_at_point()`), documented in
   [queries.md](queries.md) and covered by `tests/test_queries.cpp`.

   Overlap runs the same narrow phase the simulation does, so a query and a contact cannot
   disagree, with a containment check layered on because the solver reports nothing when one
   shape lies wholly inside another. Rays are tested against shape boundaries directly, since
   the narrow phase refuses zero-thickness segments.

   Possible follow-ups, none blocking:
   - **Accelerate with the broad-phase tree.** Queries currently scan the entity list with a
     bounding-circle rejection. The tree is only synced inside `step()`, so using it would
     answer from stale boxes between steps; doing this properly means syncing on demand.
   - **Shape sweeps.** `overlap_shape` is static. A swept version — move this shape along this
     vector, report what it would hit and when — is what character controllers want, and shares
     the time-of-impact machinery item 3 wants for CCD.
   - **Query filtering.** A category or mask so a ray can ignore whole classes of body, rather
     than the caller filtering the results.

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
   `examples/angry_boxes` covers mouse input, the draw overlay and impact strength read from
   `FxScene::contacts()`. Still open:
   - a sensor / trigger example — a goal region that fires a begin-contact event, the half of
     item 2 slice (a) no example demonstrates yet
   - ray-cast / overlap examples — still blocked on item 2 slice (b)

5. Extend mixed precision to the remaining float floors.
   Both halves of the original finding are fixed, and neither required moving storage off
   float32. The pose stays `float`; only the *residual* is banked in double, at the two
   places that need it.

   - `FxAngleWrap` no longer floors tiny rotations (`tests/test_angle_precision.cpp`).
   - Constraint corrections now go through `FxEntity::apply_pose_correction()`, which adds in
     double, stores to float, and keeps what did not fit for the next correction — the same
     carry trick `__update_pose()` already used for integration. A correction below one ulp of
     the coordinate (~`4.8e-7` at `x = 7`) used to round away entirely. Measured on an
     offset-anchor revolute motor over one second: at `dt = 1e-3` it managed **1.97e-6 rad**
     before, and **2.26 rad** after, matching what it achieves at `dt = 1e-2`. Distance from
     the origin no longer decides motor authority either.

   Remaining, same technique, lower value:
   - **Penetration correction** (`resolve_penetration`) still writes the float pose directly.
     It shifts `pose` and `prev_pose` together so the correction registers no velocity, so it
     needs the delta that actually landed — which is exactly what `apply_pose_correction()`
     returns. Penetration corrections are usually far above one ulp, so this matters only for
     deep stacks a long way from the origin.
   - **Velocity-level impulses** accumulate in float. No case has been measured where that
     costs anything; worth a look only if one shows up.

6. Build on the input layer.
   Delivered. `FxScene::input()` exposes keyboard and mouse through `FxInput`
   (`include/Fx2D/Input.h`), which knows nothing about raylib, so the same gameplay code
   compiles windowed and headless. `FxRylbRenderer` polls once per rendered frame and
   yields to ImGui when a panel has focus; a headless scene reports `available() == false`
   until user code injects state through the same producer API, which is the event-trigger
   path for scripted demos and RL agents. Documented in [input.md](input.md) and covered by
   `tests/test_input.cpp`. Mouse position is reported in scene units so picking needs no
   conversion.

   What could follow, none of it blocking:
   - **Gamepad support.** Same shape as the keyboard table: an `FxGamepadButton` enum plus
     axis state, filled from raylib's gamepad API in the renderer.
   - **Text input.** Character-stream rather than key state, for scenes that want naming or
     console entry. Deliberately excluded so far since ImGui already handles panel text.
   - **Per-step input.** Edge events currently last a whole rendered frame, which may span
     several physics steps; `input.md` documents the latch pattern that works around it. If
     that proves awkward in a real game, feed input per step instead of per frame.
   A worked playable example now exists: `examples/angry_boxes` is a mouse-driven slingshot
   that drags a ball back and topples a tower, with the mechanic itself covered headlessly by
   `tests/test_slingshot.cpp`, which injects mouse state instead of a cursor. The renderer also
   gained `set_draw_callback()` for overlays a game needs but the scene does not own — the
   slingshot band, the trajectory preview and the score are all drawn through it.

7. Opt-in multithreading, only where A/B testing shows it wins.
   The engine is single-threaded today, deliberately. It previously ran the entity
   integration and velocity-derivation loops under `std::execution::par`, and that was
   measured to be **slower at every body count tested** — 10, 50, 200, 400, 800, 1600
   and 3000, against a registry cap of 4096 — while burning up to 32x the CPU:

   | bodies | par | seq | speedup | CPU multiplier |
   |--------|--------|--------|---------|-----|
   | 10     | 0.71 ms/step | 0.22 | 3.2x | 32x |
   | 50     | 4.17   | 1.35   | 3.1x    | 23x |
   | 200    | 10.18  | 5.96   | 1.7x    | 8x  |
   | 800    | 22.63  | 19.86  | 1.14x   | 3x  |
   | 3000   | 59.27  | 48.61  | 1.22x   | 2.4x |

   There was no crossover: sequential won across the whole supported range. The cause is
   structural — the work is memory-bound over `shared_ptr`-indirected AoS entities, and it
   was dispatched once per *substep*, so 11 thread hand-offs per frame cost more than the
   arithmetic they saved. The parallel policies have been removed (`src/Scene.cpp`).

   So the target is no longer "parallelize the solver". It is **opt-in, user-controllable
   threading in the few places that can actually pay for it, each justified by an A/B
   measurement before it lands.** Concretely:

   - **Measure first, always.** Any threading change ships with a before/after on a
     realistic scene sweep (tens to thousands of bodies), reporting wall time *and* CPU
     time. A change that halves wall time at 8x CPU is usually the wrong trade for a
     library that may be one subsystem among many, and is a bad trade for RL rollouts
     where many independent sims already saturate the machine.
   - **User-controllable, off by default.** Threading should be a scene-level opt-in
     (thread count, or an explicit policy on `FxScene`) rather than baked into the step.
     A batched RL workload wants each sim single-threaded; a single large interactive
     scene may want the opposite. Only the caller knows which.
   - **Best candidate: the narrow phase.** Each pair's `collision_check` is independent
     pure geometry against const entity state, and unlike the entity loops it does real
     compute per item, so it has a plausible shot at beating dispatch overhead. Parallel
     over `broad_phase_pairs` with per-thread contact buffers concatenated **in pair
     order**, never completion order. The shared-state bits currently inside that loop
     (`wake_if_disturbed`, the step-contact buffer insert, warm-start cache lookup) hoist
     into a cheap serial pass afterwards. Prove it on a many-contact scene before adopting.
   - **Second candidate: island solve.** The XPBD position solve and velocity sweeps are
     Gauss-Seidel — sequential within a group of touching bodies, but bodies only couple
     through contacts and joints, so disconnected islands are independent. Union-find over
     contact pairs plus joints gives one task per island, and also unlocks per-island
     sleeping, which is a win even single-threaded. Payoff scales with fragmentation: many
     separate stacks approach linear speedup, one giant pile gains nothing.
   - **Not worth it on current evidence:** the entity integration and velocity-derivation
     loops. These are exactly what was measured and removed; they are memory-bound and too
     cheap per item. Revisit only if the entity layout stops being AoS `shared_ptr`.
   - **Skip until proven needed:** graph colouring within an island (Jacobi-style, how XPBD
     runs on GPUs) — only relevant for huge single islands and it changes convergence.
   - **Determinism is a hard requirement**, not a nicety: fixed reduction order for float
     accumulation, no parallel writes to `m_contact_cache` (write-back stays serial),
     concatenation in pair order. A nondeterministic sim would undermine the RL story.
   - **Out of scope for now:** SoA/SIMD batch solving of contacts (Box2D v3 style) — a
     large refactor that only matters if the above land and still leave a ceiling. Note
     leaf-level SIMD hygiene already exists: `-O3 -march=native` under `FX2D_NATIVE`, and
     32-byte-aligned `FxArray` with `__restrict` loops that auto-vectorize.

8. Push past the envelope the adversarial scenes established.
   The scenes have landed (`tests/test_adversarial.cpp`): tall stacks, pyramids, mass
   ratios, thin slivers, a restitution chain, spinning bodies, a topple test and a
   kinematic platform. Thresholds in them are measured rather than aspirational.

   **The solver passed everything thrown at it.** No correctness bug was found. Columns
   up to 15 boxes and a 5-wide pyramid hold at the default 11 substeps; 20 holds at 22.
   Raising substeps monotonically reduces sink, so the knob users reach for works. 10:1
   mass ratios are near-exact. Slivers down to 0.02 thick rest without jitter. Newton's
   cradle transfers momentum and leaves the middle balls in place. A box rides a
   velocity-driven kinematic platform. Mechanical energy never rises on an inelastic
   floor, at any spin rate tested up to 100 rad/s.

   Two behaviours were initially mistaken for bugs, recorded so the mistake is not
   repeated. A box spinning at 100 rad/s vaults metres into the air — legitimate, since it
   holds ~838 J of rotational energy and lifting 1 kg by 7 m costs 70 J; measured energy
   gain is exactly zero. It then leaves the platform sideways and lands on the scene floor,
   which looked like tunneling but is not: it was at x = 0.015, far off a platform spanning
   x in [5, 35]. Likewise a 100 kg box dropped on a five-box column scatters it flat, which
   looked like interpenetration but is a plain topple — pairwise overlap checks confirm
   every body stays separated. Both lessons are now enforced as tests: assert conserved
   energy rather than height, and assert overlap rather than final height.

   What is genuinely open:

   - **Deep penetration at extreme mass ratios.** A 1000:1 resting pair at the default 11
     substeps presses the light box exactly half its height into the ground and leaves it
     there — in place, unrotated, so this is real interpenetration rather than a topple. At
     44 substeps it does not happen. The milder 100:1 case buries 0.073 at 11 substeps and
     0.005 at 44, so penetration under load is substep-limited throughout. Worth attacking
     if extreme ratios matter; the fix direction is more position-solve authority per
     substep rather than simply more substeps.
   - **Stack height is substep-limited.** A 20-box column is stable at 22 substeps and
     collapses at 11. Since a perfectly aligned column is numerically symmetric, what
     topples it is solver noise rather than physics. Raising the default trades throughput
     for height; a better position solve would buy both.
   - **Chains under tension remain untested** — the one class from the original list not
     covered. Long revolute chains (rope, bridge) with a weight at the end, testing joint
     stretch and motor authority under load. Item 5's float-precision finding was
     discovered *by accident* in this class, which is direct evidence it holds more. Worth
     resolving the duplicate-constraint warning (`FxNamedRegistry: Item
     'base_link_Anchor' already exists.`, printed by the joint tests) first, since chain
     tests depend on joint constraints registering the way the author expects.

   The suite is marked slow and skipped when `FX2D_SKIP_SLOW_TESTS=1`, which CI sets for
   its Debug/sanitizer job only. Release runs it on every push.

## Why These Matter

- Chain colliders finish practical scene authoring for static level geometry.
- Query APIs make Fx2D more usable as an engine subsystem, not just a step-and-render loop. Contacts and events (slice a) covered the reward/game-logic half; ray and overlap queries cover the observation half.
- The collision pipeline work pays twice: hoisting the broad phase out of the substep loop removes the biggest per-frame waste, and continuous collision closes the biggest correctness gap for fast-moving bodies.
- Input hooks turned the renderer from a viewer into something a playable game can be built on, without gameplay code reaching into raylib — and the same interface drives headless scenes from scripted triggers.
- Threading is worth having only where it is measured to pay. The parallel policies the engine used to carry were slower than sequential at every body count while burning up to 32x the CPU, so the discipline — A/B first, opt-in, off by default — matters more than the parallelism itself.
- Adversarial scenes are how solver robustness is actually bought — mature engines earned their trust against tall stacks, mass ratios, and loaded chains, not through architecture; each scene added is envelope the solver provably owns. The scenes now exist, and the solver cleared them: the outcome is a measured envelope rather than a bug list.
