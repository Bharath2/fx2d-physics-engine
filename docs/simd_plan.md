# SIMD Plan

The plan of record for vectorizing the solver. Single-threaded throughout: SIMD widens what
one core does per instruction, and the layout it builds is also the prerequisite for item 7's
threading, should that ever pass its A/B gate.

## The premise

2D SIMD is not about the vec2 maths. An `FxVec2f` is two floats against 4–8 lane registers, so
vectorizing x/y pairs buys nothing. The wins come from batching **independent work** — 8
contacts or 8 bodies per instruction — which is a data-layout problem before it is an
intrinsics problem. Box2D v3 is the proof of the approach.

The step split has been re-measured at the 14x4 default, which is what Phase A was gated on.
The profiler is now a real API (`include/Fx2D/Profile.h`, CMake `-DFX2D_PROFILE=ON`) and
`scripts/bench.cpp` prints the split per scene, so this table can be regenerated rather than
recalled.

Shares on the `pile` scene at 300 bodies (807 contacts), x86-64 GCC 13, Release:

| phase | at 14x4, before this work | after Phase A | after the broad-phase work |
|---|---|---|---|
| velocity passes | 37.9% | 37.2% | **47.8%** |
| narrow phase | 30.2% | 35.7% | 31.1% |
| broad phase | 14.4% | 14.2% | 4.6% |
| integration | 5.2% | 5.4% | 7.0% |
| bookkeeping | 7.9% | 3.4% | 4.3% |
| position solve | 2.0% | 2.3% | 3.0% |
| velocity derive, constraints | 0.6% | 0.4% | 0.5% |

The first column already differed from the old 11x8 numbers the plan used to quote: velocity
passes were 38%, not 47%, and the broad phase grew with scene size — 12% at 50 bodies, 25% at
1200 — because the pair query ran once per substep. That put the broad phase ahead of Phase C
for a while, and it was done first for that reason (ToDo item 3).

With it done, the ranking has returned to what the plan originally assumed, but for measured
reasons rather than assumed ones: **the velocity passes are the step**, at 48% and rising as a
share now that the phases around them have shrunk. Nothing else is above a third, and only the
narrow phase is above 10%.

| Subsystem | Vectorizable | Blocker |
|---|---|---|
| Velocity passes | yes — the prize | Gauss–Seidel ordering: contacts sharing a body cannot share a lane. Needs graph coloring. |
| Integration, AABB refresh, sleep scan | yes, trivially | Only the layout: `shared_ptr<FxEntity>` AoS has nothing contiguous to sweep. |
| Narrow phase | partially | Independent pairs batch (circle–circle first); polygon SAT is branchy, stays scalar. |
| Broad phase tree | no | Pointer-chasing descent; leave it. |

## The architectural move: gather/scatter inside step()

The public API does not change. `FxEntity`, `shared_ptr` ownership, public `pose`/`velocity`
all stay. Instead, `FxScene::step()` (`src/Scene.cpp`) gathers active bodies into
solver-local SoA columns at entry — `x[] y[] th[] vx[] vy[] w[] invM[] invI[]` — runs every
substep against indices into those columns, and scatters back before the step callback.

Grounding in what exists:

- The registry already stores entities packed (`m_items_vec` in `include/Fx2D/Registry.h`) with
  stable packed indices, so gather is a linear sweep and contacts can carry packed indices
  instead of `shared_ptr`s inside the step. Phase A made the packed index a field on
  `FxEntity` (`packed_index()`), which is what `FxContact::body1/body2` now hold.
- `FxContact` (`include/Fx2D/Solver.h`) already caches per-substep solver data — `rA/rB`,
  the cross terms, `K_n/K_t`, the four inverse masses — added by the perf pass. Phase C is a
  transpose of exactly those fields into per-color SoA arrays, not new physics.
- `FxArray_make_aligned` (`include/Fx2D/Math.h`) provides 32-byte-aligned allocation for the
  columns.
- The mixed-precision carries (`m_pose_carry`, `m_correction_carry` in `src/Entity.cpp`)
  become two more columns; `FxCarryAdd` (`include/Fx2D/Math.h`) is already a pure function of
  scalars and vectorizes as written.
- `FX2D_NATIVE` in CMakeLists already applies `-march=native` / `/arch:AVX2`; the plan writes
  width-agnostic loops and lets the compiler pick the width, with explicit intrinsics only
  where `-fopt-info-vec` proves the compiler failed.

## Phases

Every phase gates on: the full test suite (the adversarial suite has rejected wrong solver
configurations before and is the judge again), `scripts/bench.cpp` plus the bucket profile
A/B measured back to back in one process, and the CI reproduction in both configurations.
A phase that does not pay on the bench is reverted, not rationalized.

**Phase A — SoA scaffold, still scalar. Delivered.** `FxSolverBodies`
(`include/Fx2D/Solver.h`) holds `vx/vy/w/inv_m/inv_i` columns indexed by packed entity index.
`FxScene::step` gathers them after the velocity derivation, runs `init_velocity_pass`,
`warm_start` and every `resolve_velocities` sweep against indices, and scatters back. The
material constants the sweeps used to read off two entities per pass — restitution and the two
friction coefficients — are resolved once per substep into the contact, so the innermost loop
dereferences no entity at all.

The gather sits inside the substep rather than at step entry, and that is deliberate: the
narrow phase can wake a sleeper, which changes its effective inverse mass, and the constraint
solve between them works on `FxEntity` through a virtual `evaluate`. Hoisting the gather to
step entry means converting the user-extensible constraint layer to columns too, which is a
larger change than Phase A, and the columns are only 5 floats per body per substep.

Three fixes to the same hot path landed with it, none of them vector work, all of them
measured:

- `FxNamedRegistry::for_each` allocated a fresh `vector<T*>` snapshot on every call — tens of
  heap allocations per step before any physics ran. The buffer is now a member.
- The broad-phase pair list was allocated per substep. It is now an out-parameter the substep
  loop reuses, with the tree's pair scratch held as a registry member.
- The impulse cache was an `unordered_map` hashed three times per contact per substep (warm
  start, write-back, step-buffer insert). It is now a slot vector with a free list; a contact
  carries its slot, so a pair hashes once per step.
- The broad phase resolved `entity_id -> tree node` and `entity_id -> packed index` through two
  hash maps per entity and per pair. Both now live on `FxEntity`.

Measured, `scripts/bench.cpp`, 200 steps, x86-64 GCC 13 Release, wall ms/step, before -> after:

| scene | bodies | before | after | speedup |
|---|---|---|---|---|
| settling_boxes | 200 | 7.48 | 4.48 | 1.67x |
| settling_boxes | 800 | 26.35 | 16.53 | 1.59x |
| settling_boxes | 3000 | 115.81 | 85.81 | 1.35x |
| pile | 300 | 5.55 | 4.44 | 1.25x |
| pile | 600 | 12.38 | 9.97 | 1.24x |
| pile | 1200 | 30.42 | 30.12 | 1.01x |
| stacks | 200 | 9.35 | 4.95 | 1.89x |
| stacks | 600 | 29.68 | 15.73 | 1.89x |

The columns themselves were the smaller half: they took the velocity passes from 1.91 to
1.72 ms/step at 300 bodies (-10%). The allocation and hashing removals were the larger half.
The `pile` at 1200 barely moved because it is broad-phase bound, which is the finding above.

Physics did not change: `tests/test_solver_regression.cpp` pins per-body pose and velocity
against goldens captured before the refactor, and every value still lands inside tolerance.

**Phase B — the embarrassingly parallel loops.** Integration (`FxEntity::step` /
`__update_pose` logic moves over the columns), velocity derivation, AABB refresh, sleep scan.
Width-agnostic `__restrict` loops; verify vectorization happened rather than assume.

**Phase C — colored batch velocity solve. The substantial one.** Split in two, because the
colouring and the batching have to be judged together and it was not obvious in advance that
they could not be judged apart.

**C1 — the contact graph. Delivered, and not wired in.** `FxContactGraph`
(`include/Fx2D/Solver.h`, `src/ContactGraph.cpp`) greedy-colors the broad-phase pair list so no
two pairs in a colour touch the same movable body, in a fixed deterministic order, with an
overflow group for pairs that fit no colour. Immovable bodies are excluded from the conflict
test, which matters more than anything else in the algorithm: the ground is a party to a large
share of all contacts in a pile or a stack, and if it split colours there would be nothing left
to batch. `tests/test_contact_graph.cpp` checks the invariant exhaustively rather than through
simulation.

It is **not** connected to the solver, and that is the finding. Reordering the sweep by colour
was implemented three different ways and measured every time:

| variant | pile 300 | stacks 200 | settling 200 | settling 3000 |
|---|---|---|---|---|
| unordered (baseline) | 3.28 | 4.52 | 3.79 | 49.6 |
| colour, sweep via index table | 3.89 | 5.46 | 4.70 | — |
| colour, physically sort the contacts | 3.93 | 4.99 | — | — |
| colour the pair list, narrow phase follows it | 3.66 | 4.73 | 5.80 | 84 |

Every variant is slower, and the reason is the same in each: the sweeps run
`velocity_passes` times per substep over an array of 256-byte contacts — 640KB at 3000 bodies —
and turning four sequential passes into four indirected ones costs more than the reordering can
possibly save on its own. Sorting the contacts to avoid the indirection costs about what it
saves. Sorting the *pair list* instead avoids both, but throws away the narrow phase's spatial
locality, which is worse again on sparse scenes.

**The conclusion is that C1 has no standalone win to bank, so it should not be wired in until
C2 exists.** A colored sweep only pays when a colour is solved as a batch; until then it is
pure indirection. The graph is landed and tested so C2 starts from a verified partition rather
than an unverified one.

For the record, the reordering was *better* physics where it mattered. On the five-box stack
after 150 steps, lateral drift fell from 7.6e-6 to 2.9e-6 and residual lateral velocity from
1.1e-3 to 4.8e-5, against slightly more vertical residual. The adversarial suite passed
throughout. So convergence is not the obstacle — cost is.

**C2 — the batch kernel. Delivered, and SIMD finally earned its place.**

`FxContactBatch` (`include/Fx2D/Solver.h`) transposes a colour's contacts into columns once per
substep; `resolve_velocities_batched` (`src/Constraints.cpp`) sweeps them. Scattering back is
safe precisely because a colour's contacts touch disjoint movable bodies.

Three things had to be right before any lane width mattered, and each cost a measurement to
find:

1. **A uniform slot count.** The scalar kernel bounds its inner loop by `count` and indexes with
   `(i + iter) % count`, both of which differ per lane. Two fixed slots with `(slot + iter) & 1`
   give the identical visit order -- 0,1,1,0 for a two-point manifold, 0,0 for a one-point one
   -- provided the unused slot is zeroed so it reads as inactive.
2. **Specialisation on manifold size.** Running one-point contacts through the two-slot kernel is
   correct but doubles their arithmetic, and a scene of circles is *entirely* one-point contacts.
   The first version was 14% **slower** on `pile` for exactly that reason. Each colour is now cut
   into a two-point run and a one-point run, each with its own instantiation.
3. **GCC's alias-check budget.** With the code correct and every branch turned into a float
   select, GCC still refused both hot loops: `not vectorized: no vectype for stmt`. The cause was
   not the code but `--param vect-max-version-for-alias-checks`, whose default of 10 is far below
   the ~20 columns the kernel touches. Raising it turned both loops into 32-byte vector code.

That last step is what answers the question this plan has been asking since Phase A:

| scene | scalar | restructured | + vectorised |
|---|---|---|---|
| stacks | 3.102 G | 2.623 G (-15%) | **1.938 G (-38%)** |
| pile | 10.728 G | 10.642 G (-1%) | **9.141 G (-15%)** |
| settling_boxes | 3.638 G | 3.526 G (-3%) | **3.360 G (-8%)** |

Measured in cycles, not wall time. Roughly **a third of the win is the SoA restructuring and two
thirds the vectorisation itself** -- SIMD alone is worth 26% of the `stacks` step and 14% of
`pile`. That is the first time in this plan that lane width, rather than data layout, has been
the thing that paid.

No intrinsics: width-agnostic `__restrict` loops, so the same source vectorises to AVX2 on
x86-64 and NEON on ARM, and the x86-captured goldens hold on aarch64 under qemu.

Contact ordering changes with the colour grouping, so the goldens were re-baselined; every
behavioural suite, adversarial included, passed unchanged throughout.

**Phase D — the narrow phase. Delivered, and it was not SIMD.**

The plan said "circle–circle 8-wide first". Profiling said something else. A `perf` run on the
`stacks` scene put **11.5% of the entire step inside `malloc` and `free`** -- `operator new`
with `align_val_t`, `posix_memalign` and `cfree` -- and almost all of it came from two places
that allocate in loops the engine runs thousands of times per step:

- **`sat_query`** called `B_shape->project_onto(axis, s).argmin()` once per edge. That
  expression builds two temporary `FxArray`s, each an aligned heap allocation: one for
  `vertices - origin`, one for the dot products. Box-vs-box runs SAT in both directions, so a
  single box pair cost sixteen allocations per substep. Replaced by `FxShape::min_projection`,
  a running minimum over a dot product that touches no memory at all.
- **`FxShape::set_world_pose`** built three temporaries per call -- rotated vertices, translated
  vertices, bounds -- and integration calls it for every entity of every substep. Replaced by
  an in-place form that rotates straight into the cached world vertices and accumulates the
  AABB in the same pass.

Both are bit-identical: the golden tables in `tests/test_solver_regression.cpp` did not move.

Measured, wall ms/step, x86-64 GCC 13 Release, before -> after:

| scene | bodies | before | after | speedup |
|---|---|---|---|---|
| stacks | 50 | 1.18 | 0.76 | **1.55x** |
| stacks | 200 | 4.62 | 2.89 | **1.60x** |
| stacks | 600 | 14.34 | 9.00 | **1.59x** |
| settling_boxes | 200 | 3.54 | 2.82 | 1.26x |
| settling_boxes | 800 | 13.40 | 10.01 | 1.34x |
| settling_boxes | 3000 | 48.93 | 42.62 | 1.15x |
| pile | 300 | 3.45 | 3.25 | 1.06x |

`pile` barely moves because it is circles: circle-vs-circle never enters SAT, so it was never
paying the allocation. The polygon scenes, which do, gain 1.15-1.6x.

**No SIMD was written, and on this evidence none is warranted yet.** After the fix the
allocator is gone from the profile entirely, and what is left of the narrow phase is
`compute_contact_one_way` at 19% of the `stacks` step -- branchy polygon SAT, exactly the part
the plan always said stays scalar. Circle-circle batching would target a path that is already
about ten float operations and is not visible in any profile taken.

The lesson generalises past this phase: **check the profile for allocator traffic before
reaching for lane width.** Two phases in a row now, the win has been in the data path rather
than the arithmetic.

## Expectations, stated up front

Amdahl over the measured split: Phase C plausibly cuts the velocity share 3–4x, A and B trim
the rest — a realistic overall target is **1.8–2.5x per step**, on top of the 2.5x already
banked. Not lane-width speedup; the scalar parts do not vanish.

**The portability decision is settled, ahead of Phase C rather than after it**, so the batch
kernel gets written once and proven on both ISAs the day it lands instead of ported afterwards.
`FX2D_NATIVE` stays opt-in for development, `FX2D_ARCH_BASELINE` pins a fixed ISA for shipping
builds, and every architecture flag is probed rather than assumed — `-march=native` is an error
on Apple Clang for arm64 and `/arch:` does not exist for MSVC ARM64. `-ffp-contract` is pinned
off, which is load-bearing rather than tidy: with contraction at the compiler default, the
solver regression goldens **fail on aarch64**.

The suite now passes on GCC x86-64, Clang x86-64, GCC aarch64 (under qemu), and MinGW GCC with
LTO, with `-Werror` everywhere except MSVC. That is the baseline Phase C has to keep green.

Practical consequence for Phase C: write width-agnostic `__restrict` loops over the SoA and let
each target's compiler choose the width. One source auto-vectorises to AVX2 and to NEON; hand
intrinsics would need two. Drop to an `FxSimdF` wrapper with SSE2/AVX2/NEON/scalar backends only
where `-fopt-info-vec` proves the compiler failed, never as the first move — and expect NEON's
4-wide fixed width to pay about half what AVX2's 8 does, so measure per target rather than
averaging.

Effort: B is small now that A has landed; C is two sessions; D is small if it happens at all.

## The order from here

Every phase of this plan has now been attempted. Where the step goes afterwards, share of the
step, x86-64 GCC 13 Release:

| | stacks | pile | settling_boxes |
|---|---|---|---|
| `compute_contact_one_way` (polygon SAT) | **29.8%** | 3.2% | 8.7% |
| `FxScene::step` (inlined loops, gather/scatter, bookkeeping) | 13.1% | **16.6%** | **21.5%** |
| `resolve_velocities_batched` | 11.5% | 10.5% | 5.3% |
| `FxEntity::step` (integration + world-pose refresh) | 7.8% | -- | **17.6%** |
| `FxAABBTree::insert_leaf` | -- | 4.7% | **15.6%** |
| `batch_append` + `color_pairs` (Phase C2's own overhead) | 5.9% | 11.9% | -- |

The velocity solve is no longer the problem: it was ~50% of `stacks` before C2 and is 11.5%
after. What is left, in order of what a further pass should look at:

1. **Polygon SAT.** The largest single item anywhere at 29.8% of `stacks`. This is the branchy
   case the plan has always said stays scalar, and that is probably still true edge by edge --
   but box-vs-box is a fixed four-edge problem, and the *pairs* are independent, so a batched
   box-box path is the obvious thing to try. Nothing about it needs the contact graph.
2. **`FxScene::step` itself**, 13-21%. That figure is inlined lambdas, the gather and scatter,
   the sleep scan and the contact write-back, not one hot loop -- it needs line-level
   attribution before anything is done to it.
3. **Tree churn**, 15.6% of `settling_boxes` in `insert_leaf`. Swept proxies cut this once
   already; what is left is the SAH descent, which reinserts from the root.
4. **Phase C2's own overhead**, 11.9% of `pile` between the transpose and the colouring. Worth
   revisiting only if the colouring can be made incremental across substeps.

**Phase B is not worth doing as written.** Integration is 17.6% of `settling_boxes`, but most of
that is the shape world-pose refresh -- a rotate and an AABB rebuild per body per substep -- not
the integration arithmetic the phase was aimed at. Cheaper wins there are structural: skipping
bodies that cannot move (done), or not re-rotating a shape whose angle has not changed.

## Measurement tooling added with Phase A

- `include/Fx2D/Profile.h` — phase timers with a real API (`FxProfile::ms`, `reset`, `steps`),
  compiled in only under `-DFX2D_PROFILE=ON`. Replaces the anonymous `g_prof[5]` array that had
  no way to report itself.
- `scripts/bench.cpp` — three scenes now (`settling_boxes`, `pile`, `stacks`), each reporting
  mean live contact count alongside wall and CPU time, and the phase split when the profiler is
  on. Contact count matters because body count does not predict solver load: `stacks` at 200
  bodies has 200 contacts, `pile` at 300 has 807.
- `tests/test_solver_regression.cpp` — the net that makes a speed-only refactor provable. Two
  parts: bit-exact run-to-run repeatability within one process, and golden pose/velocity tables
  for four scenes. It was checked against a deliberate convergence change (2 -> 3 normal
  iterations) and failed, so it is known to fire.
- `FX2D_PIN_FP_CONTRACT` (default on) pins `-ffp-contract=off` / `/fp:precise`, PUBLIC on
  `Fx2Dlib`. Without it, whether `a*b+c` fuses is the compiler's choice and varies by target,
  so an A/B between two builds compares two different sets of arithmetic. Measured: with
  contraction left at the default, the goldens fail on aarch64.
- `cmake/toolchains/aarch64-linux-gnu.cmake` plus qemu-user runs the whole suite on ARM without
  ARM hardware, so a portability claim is checkable locally rather than only in CI.
