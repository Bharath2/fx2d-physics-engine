# Roadmap

Fx2D is a 2D rigid-body physics engine for simulation, control and reinforcement learning,
and it works for games as well. This page lists what is being worked on and what is planned.
Plans change as measurements come in. If something here matters to you, or something you need
is missing, [open an issue](https://github.com/Bharath2/fx2d-physics-engine/issues).

## In progress

- **Closing the speed gap to Box2D v3.** On matching scenes, single-threaded at default settings,
  Fx2D takes 25-45x as long per step as Box2D 3.2 ([benchmarks/box2d](../benchmarks/box2d/)).
  Most of the gap is work Fx2D repeats in each of its 14 substeps.
- **A Python package.** `pip install fx2d` with prebuilt wheels for Linux, macOS and Windows, a
  call that steps many scenes at once for batched rollouts, and Gymnasium environments.
- **A control demo.** A small robot driven through the joint motors by a controller, runnable
  headless or in the viewer.

## Next

- **More joint types.** Distance and rope first, then weld (including breakable welds), a wheel
  joint with suspension, and pulley and gear joints. Today Fx2D has revolute and prismatic
  joints with motors, plus the mouse joint.
- **A rope and bridge builder.** Turn a polyline into capsule links joined by revolute joints,
  pinned to the world or attached to bodies, from C++ or YAML.
- **Time-of-impact collision.** Speculative contacts already catch most fast bodies. Sweeps
  will close the rest, including fast bodies against chains and edges.
- **Better queries.** Category masks for ray casts and overlaps, shape sweeps for character
  controllers, and queries accelerated by the broad-phase tree.
- **More examples.** A sensor example (a goal region that reports when something enters it)
  and ray-cast and overlap examples.

## Later

- **Opt-in multithreading**, set per scene and off by default, only where measurements show it
  pays. The likely candidates are the narrow phase and solving independent islands in parallel.
  An earlier parallel version was slower than single-threaded at every scene size tested, so
  every threading change needs a before-and-after measurement.
- **A 3D engine from the same codebase.** See the [Fx3D plan](./roadmap/fx3d.md).

## Known limitations

- Two joint types plus the mouse joint. More are listed under Next.
- In the bucket-fill stress scene, 1-2 balls in 200 end up below a 0.8-thick floor. The cause
  is not yet known; a test keeps it at 3 or fewer while it is investigated.
- At extreme mass ratios (1000:1), a light box under a heavy one sinks into the ground at the
  default substep count. More substeps fix it: 44 is enough.
- Stack height depends on substeps. Columns of up to 15 boxes stand at the default of 14
  substeps; 20 boxes need 22.
- Chains and edges do not collide with each other, and speculative contacts skip chains, so a
  fast enough body can pass through a chain.

## Recently delivered

- **Cross-platform determinism.** The same scene and inputs give bit-identical results on GCC,
  Clang and MSVC, x86-64 and ARM64, with or without native CPU tuning. Fx2D computes its own
  `sin` and `cos`, since C libraries disagree in the last bit, and every CI job checks the
  golden scenes bit for bit.
- **No required dependencies.** The physics library builds with nothing but a C++20 compiler.
  Fx2D's own vector and matrix types replaced Eigen with bit-identical results. YAML loading and
  the viewer are optional, and CMake downloads their dependencies when they are not installed,
  so a fresh clone builds without installing anything.
- **Browser playground** and a **mouse joint** for dragging bodies.
- **A 2.2-8.1x faster step** on the benchmark scenes: a broad phase that only re-walks its tree
  when a body moved, an allocation-free narrow phase, and a vectorised velocity solve that
  compiles to AVX2 and NEON from the same source.
- **ARM64 and Windows support**, with CI on GCC, Clang, Apple Clang and MSVC.
- **Chain colliders** for terrain, **spatial queries** (ray casts, overlap and point queries),
  **contact events and sensors**, **keyboard and mouse input** that also works headless, and
  **entity groups** with collision filtering and reset.
- **An adversarial test suite**: tall stacks, pyramids, mass ratios, thin slivers, spinning
  bodies and a Newton's cradle, with thresholds taken from measurements.

The measurements behind this work, and the ideas that were tried and turned down, are in the
[engineering notes](./engineering-notes.md).
