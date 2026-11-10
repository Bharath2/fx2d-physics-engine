# Fx2D against Box2D

`fx2d_vs_box2d` builds the three scenes in [`scripts/bench_scenes.h`](../../scripts/bench_scenes.h)
in Fx2D and in Box2D v3.2.0, and times both on one thread. The two engines alternate inside one
process, so the CPU clock changing under a laptop affects both alike. It prints the median
milliseconds per step for each engine and the median ratio between them.

## Run it

From the repository root:

```bash
cmake -S benchmarks/box2d -B build-box2d -DCMAKE_BUILD_TYPE=Release
cmake --build build-box2d -j
./build-box2d/fx2d_vs_box2d [rounds] [steps] [--no-sleep] [--box2d-substeps=N]
```

CMake downloads Box2D at a pinned, checksummed version and builds Fx2D from this checkout. Add
`-DFX2D_ARCH_BASELINE=avx2` to time Fx2D's AVX2 build instead of its default SSE2 one.

## What is matched, and what is not

- **The scenes.** The same bodies at the same positions and sizes, with the same masses (Box2D
  gets the density that gives each body Fx2D's mass), gravity of -10 and steps of 1/60 s.
- **Materials.** Box2D has one friction coefficient where Fx2D has two; it gets Fx2D's static
  value. Restitution is 0.1, Fx2D's default.
- **Sleeping.** On in both, as by default. `--no-sleep` turns it off in both. Box2D's threshold is
  looser, so its resting box columns fall asleep within the warm-up and Fx2D's do not.
- **Solver settings.** Each engine runs its default: Fx2D 14 substeps with 4 velocity passes each,
  Box2D 4 substeps. `--box2d-substeps=14` gives Box2D Fx2D's substep count.
- **SIMD.** Box2D chooses its SIMD width at run time and uses its 8-wide AVX2 contact solver on any
  CPU that has AVX2, even in a default build. Fx2D's default build is SSE2.
- **The workload check.** Each engine's contact count is printed to show they are solving the
  same thing. Fx2D counts touching pairs; Box2D counts its contact list, which also holds pairs
  whose bounding boxes overlap without touching.

## Results

Measured on an Intel Core i7-11800H laptop, Windows 11, MinGW GCC 15, 5 rounds of 60 steps.
Absolute times varied up to 5x between runs as the laptop throttled; the ratios, measured side
by side, held steady. Fx2D's time per step divided by Box2D's:

| | settling_boxes (200-3000) | pile (300-1200) | stacks (200-600) |
|---|---|---|---|
| Default settings, sleeping on | 18-21x | 20-39x | not comparable: Box2D's columns sleep |
| Sleeping off | 18-22x | 20-35x | 30x |

Before Fx2D held contacts through its substeps (2026-11-08), with sleeping off, the three
scenes read 23-36x, 22-44x and 43-46x. Fx2D's AVX2 build gives the same ratios as its SSE2
build, and the contact counts of the two engines agree to within 10% on every scene.

Fx2D now detects collisions mostly once per step, as Box2D does, but still runs 4 velocity
passes in each of its 14 substeps: 56 velocity sweeps per step against Box2D's 8. Fewer fail its
quality suite ([engineering notes](../../docs/engineering-notes.md#tried-and-turned-down)). Box2D also solves batches of 8 contacts at a time over
body state kept in packed arrays. Where Fx2D's step goes, from `-DFX2D_PROFILE=ON`:

| share of the step | settling_boxes (800) | pile (300, 600) | stacks (600) |
|---|---|---|---|
| velocity passes | 35% | 52%, 42% | 56% |
| narrow phase (mostly re-measuring held contacts) | 26% | 28%, 37% | 24% |
| broad phase | 16% | 4%, 6% | 1% |
| integration | 11% | 3%, 3% | 6% |
| everything else | 12% | 13%, 12% | 13% |
