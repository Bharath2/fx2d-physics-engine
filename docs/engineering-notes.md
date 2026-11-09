# Engineering notes

The measurements and decisions behind Fx2D's performance and solver work: where step time
goes, which changes paid, which ideas were tried and turned down, and how to measure a change
reliably. For planned work, see the [roadmap](./roadmap.md). For workflow, code style and the
test gate, see the [contributing guide](./contributing.md).

## How changes are measured

The benchmark is `scripts/bench.cpp` (build with `-DFX2D_BUILD_BENCH=ON`). It runs three scenes
and reports contact counts alongside time, since contacts are the solver's real workload:

- **`stacks`**: box stacks, all polygon contacts.
- **`pile`**: circles packed into a walled container, contact-dense.
- **`settling_boxes`**: a loose grid of boxes settling onto the ground, dominated by per-body
  work.

Figures on this page are CPU cycles from `perf stat -e cycles` on x86-64 with GCC 13 in Release,
unless stated otherwise. Wall time is not used; [Measuring pitfalls](#measuring-pitfalls)
explains why. Changes meant only to speed things up must leave
`tests/test_solver_regression.cpp`, which pins exact solver outputs, unchanged.

## The performance rework

The rework covered about a dozen independent changes, listed below. Against the engine before
it, on the same benchmark:

| scene | before | after | speedup |
|---|---|---|---|
| stacks | 15.35 G cycles | 1.89 G cycles | **8.1x** |
| settling_boxes | 11.29 G cycles | 4.05 G cycles | **2.8x** |
| pile | 22.11 G cycles | 10.23 G cycles | **2.2x** |

Contact counts are identical on `stacks` and `settling_boxes` and within 2% on `pile`, so this
is the same physical work done faster, not less work.

### Changes that paid

| change | effect |
|---|---|
| Removed allocations from the narrow phase (`sat_query` built two temporary arrays per edge; `set_world_pose` three per entity per substep) | 1.55-1.6x on `stacks`, 1.15-1.35x on `settling_boxes` |
| Returned shape vertices, bounding boxes and geometry by reference instead of by value | 12-18% on `stacks` on its own |
| Colour-batched, vectorised velocity solve (no intrinsics; the same source compiles to AVX2 and NEON) | 1.60x on `stacks`, 1.17x on `pile`, 1.08x on `settling_boxes` |
| Broad phase walks its tree only when a leaf moved (about 90% of substeps skip the walk in settling and piling scenes) | 1.05-1.30x on `pile`; broad phase fell from 14% of the step to 4.6% |
| Broad-phase proxies swept along velocity for the whole step | 1.4-1.6x on `settling_boxes`, 1.8-2.2x on `pile` |
| Stop SAT after the first separating axis | -6.2% on `stacks` |
| Clip only the winning SAT direction | -8.3% on `stacks` |
| Contacts and events borrow entities instead of holding `shared_ptr`s | reference-count traffic fell from 3.32% of the `pile` step to 0.09% |
| Cached contact-slot lookup per pair per step instead of per contact per substep | ~4% on `pile` |
| Per-substep solver scratch moved out of `FxContact` | `FxContact` shrank from 256 to 120 bytes |
| Skip integration for bodies that cannot move | Static level geometry no longer rebuilds its vertices and bounding box every substep |

All of these are bit-identical except the swept proxies, which change contact order and were
re-baselined; every behavioural suite, including the adversarial one, passed unchanged.

Three details of the batched velocity solve were each needed for the speedup:

1. Every contact has a uniform two-slot manifold, so no vector lane has to branch.
2. The solve is specialised on manifold size. Without that, the circle-heavy `pile` ran 14%
   *slower* than scalar code.
3. GCC's `vect-max-version-for-alias-checks` is raised from its default of 10 to 200.
   Otherwise GCC silently refuses to vectorise both hot loops.

### Replacing Eigen

`FxVec` and `FxMat` replaced the Eigen-based types with bit-identical results: a dump of every
body's pose and velocity after each of 300 steps, over 11 scenes, matched the Eigen build
exactly. Instruction counts against the Eigen build, from valgrind with GCC 13:

| build | settling_boxes (400) | pile (300) | stacks (200) |
|---|---|---|---|
| SSE2 | +0.4% | +0.4% | +1.4% |
| AVX2 (`-march=x86-64-v3`) | +0.2% | -0.4% | +1.1% |

Most of what remains is GCC now vectorising the per-point loop in `init_velocity_pass`, which it
did not attempt through Eigen's types. Instructions are not cycles; a cycle count on hardware
with counters is still owed.

## Where step time goes now

| | stacks | pile | settling_boxes |
|---|---|---|---|
| `sat_query` (polygon SAT) | **24.9%** | small | moderate |
| `resolve_velocities_batched` | 14.1% | 10.5% | 5.3% |
| `FxScene::step` (inlined loops, bookkeeping) | 9.9% | **16.6%** | **21.5%** |
| `resolve_penetration` | 8.3% | 3.1% | - |
| `FxEntity::step` (integration + world-pose refresh) | 8.1% | - | **17.6%** |
| `batch_append` + `color_pairs` (batched solve overhead) | 7.1% | **11.9%** | - |
| `FxAABBTree::insert_leaf` | - | 4.7% | **15.6%** |

The most promising targets, by expected return against effort:

1. **`FxScene::step` itself**, 10-21%. The figure is spread across inlined lambdas, the sleep
   scan and the contact write-back rather than one hot loop, so it needs line-level attribution
   first.
2. **Tree churn**, 15.6% of `settling_boxes` in `insert_leaf`. Swept proxies cut how often it
   runs; what remains is the SAH descent, which reinserts from the root.
3. **The batched solve's own overhead**, 11.9% of `pile` in the transpose and the colouring.
   Worth attacking only if the colouring can be made incremental across substeps.

## Tried and turned down

Each of these was implemented or measured and then rejected. The reasons are recorded so the
ideas can be revisited when the reason stops holding.

**Parallel entity loops.** The engine once ran integration and velocity derivation under
`std::execution::par`. It was slower than sequential at every body count tested, while using up
to 32x the CPU:

| bodies | parallel | sequential | sequential faster by | CPU multiplier |
|---|---|---|---|---|
| 10 | 0.71 ms/step | 0.22 ms | 3.2x | 32x |
| 50 | 4.17 ms | 1.35 ms | 3.1x | 23x |
| 200 | 10.18 ms | 5.96 ms | 1.7x | 8x |
| 800 | 22.63 ms | 19.86 ms | 1.14x | 3x |
| 3000 | 59.27 ms | 48.61 ms | 1.22x | 2.4x |

The work is memory-bound, and it was dispatched once per substep, so the thread hand-offs cost
more than the arithmetic they saved. The parallel policies were removed. Any future threading
will be opt-in per scene and must show a before-and-after on scenes from tens to thousands of
bodies, reporting wall time and CPU time.

**Storing the collision shape inside the entity.** Removing the `shared_ptr` indirection to the
shape was worth -4.9% on `settling_boxes` but +2.4% on `pile`, with instruction counts within
0.2%: a pure memory-layout effect. The prediction had been the reverse. Shapes behind
`shared_ptr` are allocated consecutively, so pair-heavy scenes sweep them at a 96-byte stride,
while inlined shapes sit 400 bytes apart inside entities. Neither layout wins both. The layout
worth trying next is a separate packed shape array indexed like the entities.

**Full-step swept bounding boxes, the first time.** Querying the broad phase once per step over
each body's swept box was correct (zero missed pairs against brute force across ~5M overlapping
pairs) but 22% slower on `settling_boxes`, because the narrow phase was then expensive enough
that the extra pairs cost more than the tree saved. After the narrow phase became about three
times cheaper, the same idea came back as swept proxies and was worth up to 2.2x. A rejection
measures a moment; re-test an idea when the thing that rejected it has changed.

**A rectangle fast path in SAT.** Treating boxes as oriented boxes (two axes instead of four,
overlap from half-extents) could save 12-18% on box scenes and nothing on circle scenes. It was
declined because it would change results and add a second narrow-phase path to maintain. Other
`sat_query` tweaks are worth 3-4% at best: about half its cost is loop control, not arithmetic.

**Vectorising integration.** Integration is 17.6% of `settling_boxes`, but most of it is the
world-pose refresh (a rotation and a bounding-box rebuild per body per substep), not the
arithmetic a vectorised integrator would speed up.

**Sharing lever arms between the position and velocity solves.** Not viable: the position
solve moves the bodies first, so the two compute lever arms against different poses.

## Solver envelope

The adversarial suite (`tests/test_adversarial.cpp`) covers tall stacks, pyramids, mass ratios,
thin slivers, a restitution chain, spinning bodies, a topple test and a kinematic platform. Its
thresholds are measured, not aspirational. The solver passed everything in it.

- The default of 14 substeps x 4 velocity passes is the cheapest configuration that passes the
  full quality suite, 26-33% cheaper per step than the previous 11 x 8.
- Columns of up to 15 boxes and a 5-wide pyramid hold at the default; 20 boxes hold at 22
  substeps.
- 10:1 mass ratios are near-exact. Slivers down to 0.02 thick rest without jitter. Newton's
  cradle transfers momentum and leaves the middle balls in place.
- Mechanical energy never rises on an inelastic floor at any spin rate tested, up to 100 rad/s.

Two results looked like bugs and were not. A box spinning at 100 rad/s jumps metres into the
air, which is legitimate: it holds ~838 J of rotational energy, and lifting 1 kg by 7 m costs
70 J. A 100 kg box dropped on a five-box column scatters it flat, which looked like
interpenetration but is a plain topple. The tests now assert conserved energy rather than
height, and pairwise overlap rather than final height.

Open issues:

- **Penetration at extreme mass ratios is substep-limited.** A 1000:1 resting pair at 11
  substeps presses the light box half its height into the ground; at 44 substeps it does not
  happen. 100:1 sinks 0.073 at 11 substeps and 0.005 at 44. The likely fix is more
  position-solve authority per substep rather than more substeps.
- **Stack height is substep-limited.** A perfectly aligned column is numerically symmetric, so
  what topples it is solver noise. A better position solve would allow taller stacks without
  more substeps.
- **Bodies escaping the bucket-fill floor.** 1-2 balls in 200 end up below the 0.8-thick catch
  floor, and doubling substeps does not change it, so it is not the substep-limited mode above.
  The suspected cause is corner or squeeze ejection under load. The test pins it at 3 or fewer.
- **Loaded chains are untested.** Long revolute chains with a weight on the end are the one
  adversarial class not yet covered. They arrive with the rope builder on the
  [roadmap](./roadmap.md).

## Precision

Poses are stored in `float`. Two places used to lose small corrections entirely, and both now
keep the residual in `double` and carry it to the next step:

- `FxAngleWrap` no longer rounds away tiny rotations (`tests/test_angle_precision.cpp`).
- Constraint corrections go through `FxEntity::apply_pose_correction()`. A correction smaller
  than one float step of the coordinate (~4.8e-7 at x = 7) used to vanish. On an offset-anchor
  revolute motor over one second at dt = 1e-3, the motor achieved 1.97e-6 rad before the fix
  and 2.26 rad after, matching what it achieves at dt = 1e-2.

Penetration correction still writes the float pose directly. It is usually far above the float
step, so this matters only for deep stacks a long way from the origin.

## Measuring pitfalls

- **The GCC vectoriser budget matters.** Without `--param=vect-max-version-for-alias-checks=200`,
  both hot loops of the batched solve fall back to scalar and `stacks` is ~26% slower, with no
  test failing because results are identical. If you change that kernel, check it with
  `-fopt-info-vec`.
- **The same budget versions tiny loops.** Under it GCC wraps even a two-element loop in runtime
  alias checks. When `FxVec` first replaced Eigen with plain loops over its elements,
  `init_velocity_pass` doubled in size; the element-wise operations are now unrolled at compile
  time (`fx_detail::unrolled`).
- **Wall time on a laptop is not a measurement.** The same unchanged binary measured 8.50, 11.89
  and 3.47 ms/step as the CPU moved on and off turbo. Count cycles, interleave the two
  binaries, and take minimums. Where hardware counters are unavailable (WSL, most VMs),
  valgrind's `cachegrind` counts instructions exactly: a different measure from cycles, but
  repeatable to the instruction.
- **Rebuild what you measure.** `cmake --build ... --target Fx2DTests` does not rebuild
  `Fx2DBench`.
- **Two fixes are defensive.** The overflow colour in the batched solve is solved one contact at
  a time, and `add_joint` rolls back when a constraint fails to register. Both are correct by
  construction, but no scene in the suite triggers either.

## Determinism

The enforced rule today is bitwise repeatability within one build. In practice results are
better than that: the golden scenes, and a harder scene of a dozen boxes tumbling at 4-14 rad/s
for 400 steps, come out byte-identical on GCC x86-64, Clang x86-64 and GCC aarch64. Nothing in
CI asserts that yet. Making it a guarantee across GCC, Clang and MSVC on x86-64 and ARM64 is on
the [roadmap](./roadmap.md). The conditions it depends on are under *Determinism across targets*
in the [contributing guide](./contributing.md#determinism-across-targets).
