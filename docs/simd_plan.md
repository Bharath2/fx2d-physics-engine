# SIMD Plan

The plan of record for vectorizing the solver. Single-threaded throughout: SIMD widens what
one core does per instruction, and the layout it builds is also the prerequisite for item 7's
threading, should that ever pass its A/B gate.

## The premise

2D SIMD is not about the vec2 maths. An `FxVec2f` is two floats against 4–8 lane registers, so
vectorizing x/y pairs buys nothing. The wins come from batching **independent work** — 8
contacts or 8 bodies per instruction — which is a data-layout problem before it is an
intrinsics problem. Box2D v3 is the proof of the approach.

The measured step split (bucket scene, 300 bodies, `FX2D_PROFILE` in `src/Scene.cpp`, taken
post contact-cache at 11x8): velocity passes 47%, narrow phase 25%, broad phase 12%,
integration and bookkeeping 15%, position solve 1.5%. The 14x4 default shifts weight from
sweeps toward per-substep fixed costs, so **Phase A starts by re-profiling at the current
defaults** rather than trusting these shares.

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

- The registry already stores entities packed (`m_items_vec` in `include/Fx2D/Registry.h`)
  with stable packed indices (`m_entity_idx_map`), so gather is a linear sweep and contacts
  can carry packed indices instead of `shared_ptr`s inside the step.
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

**Phase A — SoA scaffold, still scalar.** Body columns, gather/scatter, index-based contacts
inside the step; no vector code at all. Expected to win on its own by ending `shared_ptr`
chasing in the inner loops. If gather/scatter overhead eats the win at small N, that is
learned here at minimum cost. Also: re-profile at 14x4 to re-rank the remaining phases.

**Phase B — the embarrassingly parallel loops.** Integration (`FxEntity::step` /
`__update_pose` logic moves over the columns), velocity derivation, AABB refresh, sleep scan.
Width-agnostic `__restrict` loops; verify vectorization happened rather than assume.

**Phase C — colored batch velocity solve. The substantial one.** Greedy-color the contact
graph in a fixed deterministic order; pack each color's constraint data into SoA; solve
8-wide Jacobi within a color, colors in sequence (`resolve_velocities` in
`src/Constraints.cpp` becomes the batch kernel). Two known risks with existing nets:
convergence weakens slightly versus pure Gauss–Seidel, so the 14x4 study harness re-runs to
retune passes if the stacks demand it; and determinism is preserved by fixed color and batch
order with no atomics. Restitution-floor and friction-cone semantics must survive verbatim.

**Phase D — narrow-phase batching, optional.** Circle–circle 8-wide first, and only if the
post-C profile still says the narrow phase dominates. Polygon SAT stays scalar.

## Expectations, stated up front

Amdahl over the measured split: Phase C plausibly cuts the velocity share 3–4x, A and B trim
the rest — a realistic overall target is **1.8–2.5x per step**, on top of the 2.5x already
banked. Not lane-width speedup; the scalar parts do not vanish. Portability decision due at
Phase B: `FX2D_NATIVE` stays opt-in, with a fixed AVX2 baseline option for distributable
binaries. `-ffp-contract` is pinned so FMA contraction cannot silently change results
between builds.

Effort: A and B together are roughly a session; C is two; D is small if it happens at all.
