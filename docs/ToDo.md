# Fx2D Roadmap

This file tracks the next feature and robustness targets for Fx2D. Delivered
targets are removed; see git history for what already landed (shapes unified
under `vertices[] + skin_radius`, capsules/edges/rounded variants, speculative-
contact CCD, broad-phase hardening, the test suite, joint-control examples,
the `FxAngleWrap` precision fix, contacts/events/sensors, the YAML inertia
ordering fix, the adversarial scene suite, keyboard/mouse input, entity groups
with intra-group collision filtering, the chain collider with its one-sided,
ghost-vertex contact handling, the solver perf pass — per-substep contact
caching plus the measured 14x4 substep/velocity-pass default, ~2.5x per step
combined — and the elasticity default dropping to 0.1).

## How to work this file

The numbered items below are the reference detail; this section is the pickup order. The
working practice that produced everything delivered so far: measure before changing (the
benchmark is `scripts/bench.cpp` -- three scenes, reporting contact counts as well as time --
and the profiler is `include/Fx2D/Profile.h`, built with `-DFX2D_PROFILE=ON`), let the full
16-suite run judge physics changes — the adversarial suite has rejected wrong configurations
more than once, and `tests/test_solver_regression.cpp` now pins the numbers themselves for
refactors that are meant to change only speed — and reproduce CI (format + Release + Debug/ASan
with -Werror) before pushing.

## Recently delivered

These are done; they are recorded here rather than in the pending list because each one changed
how the next decision should be made.

- **Mouse joint and world-anchored constraints — delivered.** `FxMouseJoint` drags a body by a
  grab point towards a moving world target, the joint behind click-and-drag. It is the first
  constraint anchored to the world rather than to a second body: `entity2` is null,
  `FxConstraint::resolve` treats an absent second body as immovable, and the dead-reference
  sweeps no longer mistake that for a dangling pointer, so a future weld-to-world is cheap.

  The tests found a design flaw worth recording. A position constraint is an undamped spring
  here, because velocity is derived from the pose change, so every pull becomes momentum: the
  first version overshot a 10-unit drag by 8 units. A damping knob recovered only 12% of that,
  since it runs before integration and the constraint regenerates the velocity afterwards. The
  fix was `carries_velocity = false`, moving `prev_pose` with `pose` the way penetration
  recovery does, which takes overshoot to exactly zero and let the damping knob be deleted.
  Five tests in `tests/test_joints.cpp`, and the overshoot assertion discriminates: reverting
  the flag fails it at 8.08.

- **Half-wired joints reported success — fixed.** `add_joint` added the joint, looped adding its
  constraints, and ignored every one of their return values. A joint whose anchor or limit
  failed to register was reported as added while being silently weaker than asked for. It now
  rolls back the constraints already added plus the joint, and returns false. Latent rather than
  live: the only registry warnings in the suite are two deliberate ones in the group tests.

- **Cross-platform and ARM — delivered.** Architecture-aware tuning flags, every one of them
   probed rather than assumed (`-march=native` is an error on Apple Clang arm64, `/arch:` does
   not exist for MSVC ARM64); `FX2D_ARCH_BASELINE` for shipping builds; a CI matrix of six jobs
   across two ISAs and three compiler families; and `cmake/toolchains/aarch64-linux-gnu.cmake`
   so ARM is testable locally under qemu. The suite passes on GCC x86-64, Clang x86-64, GCC
   aarch64 and MinGW GCC. Two real defects fell out of building with Clang for the first time:
   53 signed-index subscripts in the AABB tree, and a pair of write-only fields on the revolute
   joint. Details in [CONTRIBUTING.md](CONTRIBUTING.md) under Portability.

- **Narrow phase — delivered, and not by batching.** Profiling found 11.5% of the step inside
   `malloc`/`free`, from `sat_query` allocating two temporary `FxArray`s per edge and
   `set_world_pose` allocating three per entity per substep. Both are now allocation-free and
   bit-identical, worth **1.55-1.6x on `stacks`** and 1.15-1.35x on `settling_boxes`. `pile` is
   circles and never entered SAT, so it barely moves. No SIMD was written and none is currently
   justified: the allocator is gone from the profile, and what remains is branchy polygon SAT.
   See [simd_plan.md](simd_plan.md) Phase D.

- **Duplicated SAT sweep — removed.** `sat_query` and `sat_gap_query` shared their whole
  per-edge body and differed only in what they accumulated: the first keeps the largest gap and
  exits early on a separating axis, the second keeps the smallest and never exits. Both now call
  one `sat_axis` helper. Measured interleaved, it is a wash on cycles; the first non-interleaved
  reading claimed a 4.4% regression and was ordering noise.

- **Narrow phase, two exact wins — delivered. ~14% off the `stacks` step.** Both come from
  removing work the two-way polygon test was doing and discarding, not from batching:

  | change | stacks | pile | settling_boxes |
  |---|---|---|---|
  | stop after the first separating axis instead of testing both directions | **-6.2%** | -4.2% | -1.2% |
  | split SAT from clipping so only the winning direction is clipped | **-8.3%** | -1.5% | +1.4% |

  `collision_check` has to try both directions to find the smaller penetration, but it was
  running a full SAT *and* a full clip each way and throwing one result away -- and it computed
  the second direction even when the first had already found a separating axis. The clip is the
  expensive half: two normalisations plus the edge clip itself. `polygon_contact_from_sat` is
  now separate and runs once. Chains still take the general path, since `is_polygon()` and
  `is_chain()` are mutually exclusive.

  The `settling_boxes` figure is inside the noise band this machine produces; the two other
  scenes moved consistently.

- **SIMD Phase C2 — delivered. 1.60x on `stacks`, 1.17x on `pile`, 1.08x on `settling_boxes`.**
  The colour-batched velocity solve, and the first change in this plan where vector width rather
  than data layout was the thing that paid: roughly a third of the win is the SoA transpose, two
  thirds the vectorisation itself. Three non-obvious requirements, each of which cost a
  measurement to find -- a uniform two-slot manifold so no lane has to branch; specialisation on
  manifold size, without which the circle-heavy `pile` was 14% *slower* than scalar; and raising
  GCC's `vect-max-version-for-alias-checks` from its default of 10, which was silently refusing
  both hot loops. No intrinsics, so the same source vectorises to AVX2 and to NEON. Detail in
  [simd_plan.md](simd_plan.md).

- **Three small exact wins in the hot loops — delivered.** All bit-identical, all measured in
  cycles rather than wall time:

  | change | stacks | pile | settling_boxes |
  |---|---|---|---|
  | cache both bodies' velocities in locals across `resolve_velocities` | **-3.0%** | -0.4% | -1.5% |
  | reuse the AABB tree's sibling-search stack instead of allocating per insert | +0.4% | **-1.4%** | **-1.1%** |
  | write `min_projection`'s inner loop in scalars instead of Eigen expressions | **-0.9%** | -0.8% | -0.5% |

  The first is the interesting one. `resolve_velocities` applies up to six impulses per call, and
  because `FxSolverBodies` is a mutable reference indexed by runtime values, the compiler had to
  assume each store might alias the arrays and reload all six velocity components afterwards.
  Only that one contact touches those two bodies within the call, so hoisting them into locals
  and writing back once is exact.

  The second was the fifth instance of the same allocation trap: `find_best_sibling` built and
  destroyed a `std::vector` search stack on every tree insertion, and a body that keeps escaping
  its fat box is reinserted every substep.

- **Storing the collision shape by value — tried and rejected on measurement.** `FxEntity` holds
  its collision shape behind a `shared_ptr`, so refreshing the world pose chases a pointer to a
  separate 96-byte allocation once per entity per substep -- 5.3% of the `settling_boxes` step by
  line-level profile. Inlining the shape removes that indirection. It was implemented, passed all
  suites with goldens unchanged, and then taken back out.

  Wall-clock A/B was worthless: the same unchanged binary measured `settling_boxes/200` at 8.50,
  11.89 and 3.47 ms/step across three windows, because the laptop moves on and off turbo. Cycle
  counts settled it, being frequency-independent:

  | scene | shared_ptr | inline | delta |
  |---|---|---|---|
  | pile | 11.76 Gcycles | 12.05 Gcycles | **+2.4%** |
  | settling_boxes | 4.714 Gcycles | 4.483 Gcycles | **-4.9%** |

  Instruction counts matched to within 0.2%, so this is purely memory layout. A wash overall,
  against `FxEntity` growing 312 -> 400 bytes and an API change that drops a lifetime guarantee.

  **The mechanism turned out to be the reverse of the hypothesis**, which is the part worth
  keeping. The prediction was that inlining would help the contact-dense `pile` (fewer bodies,
  many shape lookups) and hurt body-heavy `settling_boxes`. The opposite happened: shapes behind
  `shared_ptr` are allocated consecutively, so a pair-heavy scene sweeping many shapes gets a
  96-byte stride through a dense region, while inlining spreads them 400 bytes apart inside the
  entities. Integration, which touches an entity and its shape together, prefers them fused.
  Neither effect dominates, so there is no layout that wins both.

  If it is ever revisited, the shape to aim for is a separate packed shape array indexed by the
  entity's packed index -- dense for the narrow phase *and* free of the pointer chase -- rather
  than a choice between the two current layouts.

- **Broad-phase proxies swept over the step — delivered, after being rejected once.** The tree
  now fattens each moving body's proxy along its velocity for the whole step, so a body at
  constant velocity stays inside the box the tree already holds and needs no reinsertion for the
  rest of the step. Worth **1.4-1.6x on `settling_boxes` and 1.8-2.2x on `pile`**, a wash on
  `stacks`.

  This exact idea was tried earlier and reverted: it inflates the pair list, and at the time the
  narrow phase was expensive enough that the extra pairs cost more than the tree saved. What
  changed is that the narrow phase got roughly three times cheaper in between (the allocation
  and return-by-value fixes below), so the trade reversed. **Re-test rejected ideas when the
  thing that rejected them has moved** -- that is the reusable lesson, and it is worth more than
  the speedup.

  It changes contact ordering, so the goldens were re-baselined; every behavioural suite,
  including the adversarial one, passed unchanged.

- **Integration skipped for immobile bodies — delivered.** A body with no inverse mass or
  inertia and no velocity cannot change pose, but static level geometry never sleeps
  (`tick_sleep` exempts zero-mass bodies deliberately), so it re-rotated every vertex and
  rebuilt its AABB every substep forever. Now skipped, exactly: the check compares against the
  pose the shape was last built at, so a body moved by writing `pose` directly still updates.
  Invisible on the benchmarks, which have one or three static bodies; it is scenes built from
  static geometry that pay this, which is most real ones.

- **Per-substep scratch moved out of `FxContact` — delivered.** Lever arms, effective masses,
  the restitution target and the mixed material constants are rebuilt every substep and never
  read outside it, but they were carried through every copy of the contact into the step buffer.
  Moved to a parallel `FxContactSolverData` array: **`FxContact` went from 256 to 120 bytes**,
  and the array is also the layout a batched solve would want. Bit-identical.

  Its sibling idea -- sharing the lever arms between `resolve_penetration` and
  `init_velocity_pass` -- was investigated and is **not viable**: the position solve runs
  earlier in the substep and moves the poses itself, so the two compute `rA`/`rB` against
  genuinely different poses. Sharing them would be wrong, not merely awkward.

- **Return-by-value in the hot path — delivered.** Four accessors were handing back copies of
  things the caller only read: `FxEntity::bounding_box()`, `FxEntity::collision_geometry()` and
  `visual_geometry()`, and `FxShape::vertices()`. The last is the one to remember -- it copied
  every vertex into a fresh aligned allocation, and because every call site spells it
  `const auto& v = shape->vertices()`, the copy was invisible at the point of use while the
  narrow phase made several per pair per substep. Returning by reference is worth **12-18% on
  `stacks`** on its own and is bit-identical.

- **Contact slot lookup — delivered.** `bind_contact_slot` hashed the pair key once per contact
  per substep. The broad-phase pair list is stable across a step, so the resolved slot is now
  cached against the pair index and the hash runs once per pair per step. Worth ~4% on `pile`,
  matching its profile share.

- **Reference-count traffic — delivered.** `FxContact` and `FxContactEvent` now borrow their
   entities as raw pointers instead of owning them through `shared_ptr`, and
   `FxEntity::collision_geometry()` / `visual_geometry()` return by reference instead of by
   value. The second of those was by far the larger source: the accessor was called several
   times per pair per substep and each call was an atomic increment and decrement.
   `_Sp_counted_base::_M_release` fell from **3.32% of the `pile` step to 0.09%**. Physics is
   bit-identical.

   The lifetime guarantee those `shared_ptr`s provided -- an entity deleted mid-flight is still
   nameable by the end-contact event that reports its separation -- is now explicit
   (`FxScene::pin_contact_entities`) and, for the first time, tested. It was previously relied
   on by a comment and nothing else.

## Pending, in order

The whole SIMD plan has now been attempted end to end; what remains is ordinary engine work plus
a re-ranked performance list. Physics features come first because the engine is fast enough that
the next users are more likely to be blocked by a missing joint than by a slow step.

1. **The rope thread** (item 10): distance joint -> FxChain dynamic mode -> bridge demo ->
   chains-under-tension tests. One connected piece of work; each stage is useful alone, and the
   end closes the last untested adversarial class from item 8.
2. **The floor escape** (detail in item 8): the only unexplained defect. 1-2 balls per 200
   through the 0.8-thick catch floor, substep-independent, pinned at <=3 by the bucket test.
3. **Weld and wheel joints** (the rest of item 9). The mouse joint has landed and brought
   world-anchored constraints with it, so a weld-to-world is now cheap; the truck example still
   hand-assembles what a wheel joint should give it.
4. **`FxScene::step` line-level attribution.** 10-21% of the step depending on scene, but that
   is inlined lambdas, the sleep scan and the contact write-back rather than one hot loop.
   Measure before touching it -- the function-level figure has already misled once.

   Two performance items were **examined and declined**, both recorded so they are not
   re-litigated blind. `sat_query` is the largest single item at 24.9% of the `stacks` step, but
   annotation shows about half of that is loop control and its compare, not arithmetic; the
   bit-exact options are worth 3-4% at best. The one large win there is a **rectangle fast path**
   using the OBB formulation -- two axes per box instead of four, overlap from half-extents --
   worth perhaps 12-18% on box scenes and nothing on the circle-heavy `pile`. A shape survey
   confirmed the premise (100% of `stacks` and `settling_boxes` contacts are four-vertex polygon
   pairs, against 1% for `pile`), but it would be the only change contemplated this session to
   move results, and it adds a second narrow-phase path. See [next_steps.md](next_steps.md).

5. **Tree-accelerated queries** (item 2 follow-up) once query volume justifies it.
6. **Time-of-impact CCD** (item 3) -- also what lets fast bodies hit chains and edges.
7. **Solver grid diagonal** (small): 11x5, 12x5 and 13x4 were never measured, and the 14x4 study
   predates every solver change since. Worth re-running the harness rather than trusting it.
8. **Threading** (item 7) -- still gated on A/B evidence that has so far said no. Phase C2's
   colour partition is what a parallel solve would start from, and it now exists.

## Priority Targets

1. Chain / polyline colliders — delivered.
   `FxShapeType::Chain`, built with `FxShape::make_chain()` or the YAML `chain:` key, is an open
   polyline of at least 3 points authored as one entity. It resolves to the deepest contact any
   of its segments makes, each handed to the existing edge routines, so it adds no new geometry.
   Covered by `tests/test_chain.cpp` and documented in [scene_yml.md](scene_yml.md).

   It inherits the edge limitations as expected: chain-vs-chain and chain-vs-edge produce no
   contacts, and chains are skipped by speculative-contact CCD, so a fast enough body tunnels.
   Lifting that is item 3's time-of-impact work, not a chain problem.

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
   geometry.

   **Broad-phase efficiency — delivered.** The tree itself was always sound (SAH-guided dynamic
   AABB tree, fat boxes, dual-tree pair descent); it was driven wastefully, with
   `get_broad_phase_pairs()` running per *substep*, so every frame paid 14 tree syncs and 14
   full pair walks.

   The query is now split in two (`include/Fx2D/Registry.h`). `sync_broad_phase()` still runs
   every substep — bodies moved, the tree must know — and returns whether any leaf was
   inserted, removed, or reinserted. `collect_broad_phase_pairs()` walks the tree, and runs
   only when that answer is yes. If no leaf moved, the tree is the same tree and the walk
   would rebuild the identical list in the identical order, so skipping it changes nothing.
   Measured skip rate: about 90% of substeps in settling and piling scenes, a third when 1600
   boxes are still in free fall. Sleeping-pair and tight-AABB filtering moved into the narrow
   phase, since the list now outlives the substep that built it.

   The result is exact rather than approximate — the golden tables in
   `tests/test_solver_regression.cpp` did not move — for 1.05-1.30x on `pile` and a wash
   elsewhere, taking the broad phase from 14% of the step to 4.6%.

   Two things went with it: the tree-node and packed indices moved onto `FxEntity` (they were
   two hash lookups per entity and per pair), and `FxEntity::bounding_box()` now returns by
   reference — returning the `FxArray` by value meant every narrow-phase pair paid two aligned
   heap allocations, and removing that alone cut the narrow phase by 37%.

   **Tried and reverted: full-step swept AABBs.** The original plan here was to query once per
   step over `combine(aabb, aabb + velocity * dt_full)`, making the list valid by construction.
   It works and it is correct — an audit against brute force found zero missed pairs across
   ~5M overlapping pairs — but in a scene of bodies in free fall the swept boxes are long, the
   list fills with pairs that never touch, and the narrow phase pays for them once per substep.
   Measured 22% slower on `settling_boxes` at 400 bodies, and it changed contact ordering, so
   results moved in chaotic scenes for no gain. The fat-box invariant gets the same coverage
   guarantee for free, because the tree was already fattening every box it stored.

   **Continuous collision.** Speculative contacts are done
   (`FxEntity::enable_ccd`, `FxSolver::speculative_contact_check()`, YAML
   `ccd:` key). Remaining, to reduce tunneling further for fast movers:
   - time-of-impact style sweeps
   - fast-body or bullet-style handling for selected entities

4. Add more examples and docs around newer features.
   `examples/angry_boxes` covers mouse input, the draw overlay and impact strength read from
   `FxScene::contacts()`; `examples/chain_terrain` covers chain colliders and spawning entities
   at runtime. Still open:
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
   - **SoA/SIMD batch solving** is no longer out of scope: it is the plan of record, see
     [simd_plan.md](simd_plan.md). Its gather/scatter layout and colored contact graph are
     also the prerequisites any future threading starts from.

8. Push past the envelope the adversarial scenes established.
   The scenes have landed (`tests/test_adversarial.cpp`): tall stacks, pyramids, mass
   ratios, thin slivers, a restitution chain, spinning bodies, a topple test and a
   kinematic platform. Thresholds in them are measured rather than aspirational.

   **The solver passed everything thrown at it.** No correctness bug was found. Columns
   up to 15 boxes and a 5-wide pyramid hold at the default configuration; 20 holds at 22
   substeps. The default became 14 substeps x 4 velocity passes after a measured study:
   fastest configuration passing the full quality suite, 26-33% cheaper per step than the
   old 11x8.
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

   - **Deep penetration at extreme mass ratios.** A 1000:1 resting pair at 11
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

   One open finding from the bucket-fill scene (150-200 balls raining into a floating
   container): the walls and bucket bottom always hold, but 1-2 balls per 200 end up below
   the 0.8-thick catch floor, and doubling substeps does not remove it -- so it is not the
   substep-limited penetration mode. Suspected corner or squeeze ejection under pile load;
   the test pins the measured envelope at <= 3 until the mechanism is found.

   The suite is marked slow and skipped when `FX2D_SKIP_SLOW_TESTS=1`, which CI sets for
   its Debug/sanitizer job only. Release runs it on every push.

9. Broaden the joint set.
   Two joint types against Box2D's eight-plus is the largest practical gap for anyone
   building a real game on Fx2D. The machinery is already in place — FxJoint composes
   constraints from the existing kernel, motors and PID come from the base class, and the
   constraint-naming scheme keeps many joints on one rig safe — so each new type is one or
   two constraint formulations plus tests. In rough order of value:
   - **Distance / rope joint** — fixed or maximum separation between two anchors; also the
     building block the FxChain dynamic mode wants.
   - **Mouse joint** — a soft spring from a world point to a body anchor, paired with
     `entity_at_point()` for click-dragging; every editor and demo wants it.
   - **Weld joint** — locks relative pose entirely; breakable variants enable destruction.
   - **Wheel joint** — revolute plus a sprung suspension axis; the truck example currently
     fakes this with hand-assembled constraints.
   - **Pulley and gear** — ratio constraints across two joints; niche but classic.

10. FxChain dynamic mode — the rope builder.
   The agreed design, deferred from the chain work: `FxChain` is a builder spec, not a
   runtime type. A polyline plus a mode. Static mode already exists (the chain collider);
   dynamic mode emits an entity group of capsule links joined by revolute joints along the
   same polyline. The spec carries the repeatable link properties (shape, mass, inertia
   explicit or computed, friction, elasticity — default it to something inert), the joint
   parameters (compliance or stiffness, optional angle limits, optional motor), and per-end
   anchoring: pinned to the world, attached to a named entity, or free. A `chains:` YAML
   section maps onto the same spec. Naming follows the group scheme: `<name>_<i>` links,
   `<name>_j<i>` joints, constraints `<joint>_<Type>`.

   Prerequisites all landed: groups give membership, bulk delete and intra-group collision
   filtering; the constraint-naming fix makes many joints on one rig safe; a distance joint
   (item 9) is the natural first link type before capsules-plus-revolutes.

   Expectations to hold the tests to: joints are maximal-coordinate, so a loaded rope will
   stretch — compliance is the dial, and the tests should pin measured stretch at a chosen
   compliance rather than assert zero. This is where item 5's float-precision finding was
   first stumbled on, so surprises are likely and finding them is the point. Exit: a bridge
   demo with balls dropped on it, and chains-under-tension in the adversarial suite.

## Why These Matter

- Chain colliders finish practical scene authoring for static level geometry.
- Query APIs make Fx2D more usable as an engine subsystem, not just a step-and-render loop. Contacts and events (slice a) covered the reward/game-logic half; ray and overlap queries cover the observation half.
- The collision pipeline work pays twice. The broad-phase half is done: the tree is still synced every substep, but it is only *walked* when a proxy actually moved, which removed the biggest per-frame waste without moving a single result. Continuous collision is the other half, and still the biggest correctness gap for fast-moving bodies.
- Input hooks turned the renderer from a viewer into something a playable game can be built on, without gameplay code reaching into raylib — and the same interface drives headless scenes from scripted triggers.
- Threading is worth having only where it is measured to pay. The parallel policies the engine used to carry were slower than sequential at every body count while burning up to 32x the CPU, so the discipline — A/B first, opt-in, off by default — matters more than the parallelism itself.
- Adversarial scenes are how solver robustness is actually bought — mature engines earned their trust against tall stacks, mass ratios, and loaded chains, not through architecture; each scene added is envelope the solver provably owns. The scenes now exist, and the solver cleared them: the outcome is a measured envelope rather than a bug list.
