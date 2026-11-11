# Roadmap

Fx2D is a deterministic 2D rigid-body physics engine for reinforcement learning and control,
used from Python or C++, and it works for games as well. This page lists what is being worked on and what is planned.
Plans change as measurements come in. If something here matters to you, or something you need
is missing, [open an issue](https://github.com/Bharath2/fx2d-physics-engine/issues).

## In progress

- **Closing the speed gap to Box2D v3.** On matching scenes, single-threaded at default settings,
  Fx2D takes 18-35x as long per step as Box2D 3.2 ([benchmarks/box2d](../benchmarks/box2d/)).
  Holding contacts through the substeps closed part of the gap; the rest is a solver that needs
  14 substeps with 4 velocity passes to keep tall stacks standing.
- **The first release on PyPI.** The wheels for Linux, macOS and Windows are built and tested
  in CI; publishing them makes `pip install fx2d` work without a compiler.
- **Rendering for environments.** `render_mode="rgb_array"` frames from Python, so a training
  run can record videos of its agent.
- **More environments.** A walker driven through its joint motors, and tasks that sense their
  surroundings with ray casts.
- **Less Python per step.** Batched reads of observations across scenes, so vector
  environments of cheap scenes such as CartPole gain as much from parallel stepping as
  contact-rich ones such as Push do.

## Next

- **More joint types.** Weld (including breakable welds), a wheel joint with suspension, and
  pulley and gear joints. Today Fx2D has revolute, prismatic, distance and rope joints with
  motors, plus the mouse joint.
- **A rope and bridge builder.** Turn a polyline into links joined by distance joints, pinned
  to the world or attached to bodies, from C++ or YAML. `examples/playground` builds its bridge
  this way by hand: a group for the links, a distance joint per pair and an anchor at each end.
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

- Four joint types plus the mouse joint. More are listed under Next.
- A large one-shot `set_limits()` change on a distance joint is solved inside one substep and
  can carry the load past the anchor, so reel a rope in gradually. A rigid bridge deck laid
  along an arc can only deepen that arc by the slack it was drawn with, so a bridge that should
  dip under load needs compliance.
- In the bucket-fill stress scene, 1-2 balls in 200 end up below a 0.8-thick floor. The cause
  is not yet known; a test keeps it at 3 or fewer while it is investigated.
- At extreme mass ratios (1000:1), a light box under a heavy one sinks into the ground at the
  default substep count. More substeps fix it: 44 is enough.
- Stack height depends on substeps. Columns of up to 15 boxes stand at the default of 14
  substeps; 20 boxes need 22.
- In `examples/playground`, the crate stack, with a hexagon and a ball resting against it from
  the ramp, creeps sideways at 0.2-0.35 cm/s and never sleeps. The resting-stability suite
  measures sinking, not lateral drift, so nothing catches it yet.
- Chains and edges do not collide with each other, and speculative contacts skip chains, so a
  fast enough body can pass through a chain.

## Recently delivered

- **Distance and rope joints.** `FxDistanceJoint` holds two anchors within
  `[min_length, max_length]`: equal limits give a rigid link, a zero minimum a rope that hangs
  slack, and `compliance` a stretch of exactly `m g c` under a load. Rotation stays free, so a
  chain of them is a rope rather than a jointed arm, and the motor drives the separation in all
  three control modes. YAML `type: distance` and `type: rope`; the playground builds a wrecking
  ball and a plank bridge from them.
- **A Python package with Gymnasium environments.** Scenes, bodies, joints, contacts and ray
  casts from Python, NumPy views of state, `step_many` for stepping scenes in parallel with the
  GIL released, and `fx2d/Push-v0`, `fx2d/CartPole-v0` and `fx2d/Pendulum-v0` with vector
  versions for `gym.make_vec`. 256 copies of Push step 2.7x as fast as through Gymnasium's
  `SyncVectorEnv` on an 8-core laptop.
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
