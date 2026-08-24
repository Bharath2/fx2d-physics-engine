# Next Steps

A session handoff, not a roadmap. [ToDo.md](ToDo.md) remains the canonical list of what the
engine should grow next; this file records where the work stopped, what state the tree is in,
and what the evidence says to do about it. If the two ever disagree, ToDo.md wins.

## Where the tree stands

Nothing is committed. The working tree holds around **30 modified files plus 8 new ones**,
covering roughly a dozen independent changes: the broad-phase rework, the batched velocity
solve, cross-platform and ARM support, several allocation and return-by-value removals, the
mouse joint, three new test suites, and three deliberate golden re-baselines.

Verified green on GCC x86-64, Clang x86-64, GCC aarch64 (under qemu), MinGW/Windows, and the
Debug ASan+UBSan build. `clang-format --check` clean, and no comment block over three lines.

Measured against the session's starting commit, same benchmark, in CPU cycles:

| scene | before | after | speedup |
|---|---|---|---|
| stacks | 15.35 G | 1.89 G | **8.1x** |
| settling_boxes | 11.29 G | 4.05 G | **2.8x** |
| pile | 22.11 G | 10.23 G | **2.2x** |

Contact counts are identical on `stacks` and `settling_boxes` and within 2% on `pile`, so this
is the same physical work done faster, not less work.

## 0. Split the change set into commits — do this first

This is the most pressing item, ahead of any new code.

A single commit of this size is close to unbisectable, and most of the value of the work is in
*why* each change landed — which measurement justified it, what was tried and rejected first.
That reasoning survives only if each change carries it. Suggested boundaries, each of which
builds and passes on its own:

1. Measurement tooling: `FxProfile`, the three-scene benchmark, the solver regression suite and
   its goldens.
2. Cross-platform: architecture-aware tuning flags, `FX2D_ARCH_BASELINE`, the aarch64 toolchain
   file, the CI matrix. Includes the two defects Clang found (signed subscripts in the AABB
   tree, write-only fields on the revolute joint).
3. Allocation and return-by-value removals: SAT projections, `set_world_pose`, `vertices()`,
   `bounding_box()`, `collision_geometry()`, the tree's sibling-search stack.
4. Broad phase: walk only when the tree changed, then full-step swept proxies. **Re-baselines
   the goldens.**
5. Solver plumbing: `FxSolverBodies` columns, borrowed entity pointers plus the pin list,
   `FxContactSolverData`, the pair-slot cache, the immobile-body skip.
6. Phase C2: the contact graph, `FxContactBatch`, the batched kernel, the GCC vectoriser budget.
   **Re-baselines the goldens.**
7. Narrow phase: separating-axis early-out, the SAT/clip split, the shared `sat_axis`.
8. Mouse joint and world-anchored constraints, including the `add_joint` rollback fix.
9. Documentation.

## 1. Features, not performance

The engine is 2.2-8.1x faster than it was. The next person building on it is far more likely to
be blocked by a missing joint than by step time.

- **The rope thread.** Distance joint -> `FxChain` dynamic mode -> bridge demo ->
  chains-under-tension tests. One connected piece of work, each stage useful alone, and the end
  closes the last untested class from the adversarial list.
- **The floor escape.** 1-2 balls per 200 pass through the 0.8-thick catch floor,
  substep-independent, currently pinned by a test rather than understood. It is the only
  unexplained defect in the engine.
- **Weld and wheel joints.** The mouse joint has landed and brought world-anchored constraints
  with it, so a weld-to-world is now cheap to add.

## 2. If performance work continues anyway

Where the step goes now, as a share of each scene:

| | stacks | pile | settling_boxes |
|---|---|---|---|
| `sat_query` (polygon SAT) | **24.9%** | small | moderate |
| `resolve_velocities_batched` | 14.1% | 10.5% | 5.3% |
| `FxScene::step` (inlined loops, bookkeeping) | 9.9% | **16.6%** | **21.5%** |
| `resolve_penetration` | 8.3% | 3.1% | - |
| `FxEntity::step` (integration + world-pose refresh) | 8.1% | - | **17.6%** |
| `batch_append` + `color_pairs` (C2's own overhead) | 7.1% | **11.9%** | - |
| `FxAABBTree::insert_leaf` | - | 4.7% | **15.6%** |

Ranked by expected return against effort:

1. **`FxScene::step` itself**, 10-21%. That figure is inlined lambdas, the sleep scan and the
   contact write-back, not one hot loop — it needs line-level attribution before anything is
   done to it. The function-level number has already misled once.
2. **Tree churn**, 15.6% of `settling_boxes` in `insert_leaf`. Swept proxies cut the frequency
   once; what remains is the SAH descent, which reinserts from the root.
3. **Phase C2's own overhead**, 11.9% of `pile` between the transpose and the colouring. Worth
   attacking only if the colouring can be made incremental across substeps.

**`sat_query` is the largest single item but is not recommended.** Annotating it shows about half
its cost is loop control and the compare feeding it, not arithmetic — four short iterations with
a runtime bound. The bit-exact options are poor trades: unrolling the `n == 4` case duplicates
the loop body for perhaps 3-4% of the step, and removing the early-exit branch would cost more
than it saves, since that early-out was itself worth 6.2%.

The one large win there is a **rectangle fast path** — the OBB formulation needs two axes per
box instead of four and reads overlap from half-extents rather than projecting vertices, which
is worth perhaps 12-18% on box scenes and nothing on `pile`. A shape survey confirmed 100% of
`stacks` and `settling_boxes` contacts are four-vertex polygon pairs, against 1% for `pile`. It
was **considered and declined**: it is the only change contemplated this session that would move
results, and it adds a second narrow-phase path to maintain.

**Phase B of the SIMD plan is not worth doing as written.** Integration is 17.6% of
`settling_boxes`, but most of that is the shape world-pose refresh — a rotate and an AABB rebuild
per body per substep — not the integration arithmetic the phase was aimed at.

## 3. Things that will bite whoever picks this up

- **The GCC vectoriser budget is load-bearing.** Without
  `--param=vect-max-version-for-alias-checks=200`, both hot loops in the batched solve silently
  fall back to scalar and `stacks` is ~26% slower, **with no test failing** — the results are
  identical either way. If you touch that kernel, re-check with `-fopt-info-vec`.
- **Wall-clock timing on a laptop is not a measurement.** The same unchanged binary was observed
  at 8.50, 11.89 and 3.47 ms/step for identical code. Use `perf stat -e cycles`, interleave the
  two binaries, and take minimums. Several changes this session were nearly reported backwards
  because of this, including one regression that first looked like a 36% win.
- **Rebuild what you measure.** `cmake --build ... --target Fx2DTests` does not rebuild
  `Fx2DBench`. More than one confusing result traced back to a stale benchmark.
- **A rejection is a measurement of a moment.** Full-step swept proxies were implemented,
  measured, and reverted early on; re-tried after the narrow phase got three times cheaper, the
  same change was worth up to 2.2x. Record why something was rejected, and revisit when that
  reason stops holding.
- **Two fixes here are defensive, not reproduced.** The overflow colour is solved one contact at
  a time, and `add_joint` rolls back when a constraint fails to register. Both are correct by
  construction; neither could be made to fire in any scene in the suite, and both are documented
  as such where they live.

## 4. Determinism, as measured

The enforced rule is bitwise repeatability *within one process*. Across builds the project only
promises tolerances. In practice it currently does better: the golden scenes, and a deliberately
harder one of a dozen boxes tumbling at 4-14 rad/s for 400 steps, come out **byte-identical** on
GCC x86-64, Clang x86-64 and GCC aarch64 under qemu.

That is an observed property, not a guarantee, and nothing in CI asserts it. See
[CONTRIBUTING.md](CONTRIBUTING.md) under *Determinism across targets* for the conditions it
depends on and what would break it.
