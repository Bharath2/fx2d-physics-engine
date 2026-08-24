# Contributing to Fx2D

Contributions are welcome! Please follow the existing code style and conventions.

- **Bugs & feature requests:** Open a new issue describing the problem or
  proposal clearly (steps to reproduce, or the rationale/use case).
- **Working on features/fixes:** Create a new branch from `main`, commit your
  changes, and open a pull request referencing the issue.
- Run `./scripts/format.sh` before opening a PR; `./scripts/lint.sh` should
  pass. Both are gated in CI on every push and pull request.
- **Looking for something to work on?** The roadmap of open targets lives in
  [docs/ToDo.md](ToDo.md) — each item records the motivation, the
  relevant code paths, and the suggested approach.

## Linting

Style and light static analysis (clang-format, cppcheck, clang-tidy) are gated
in CI (`.github/workflows/lint.yml`):

```bash
./scripts/format.sh           # rewrite sources to scripts/.clang-format
./scripts/list_sources.sh     # what the tools sweep; --core for the physics TUs
./scripts/format.sh --check   # dry-run (fails if anything would change)
./scripts/lint.sh             # format check + cppcheck + clang-tidy
./scripts/lint.sh --no-tidy   # skip clang-tidy (fastest local gate)
```

The clang-format and clang-tidy configs live in `scripts/`, so the wrapper scripts pass
them explicitly rather than relying on the tools searching upward from each file.

`lint.sh` generates a minimal `compile_commands.json` for clang-tidy via
`./scripts/gen_compile_db.sh` (physics core only, no raylib), and cppcheck
reads its suppressions from `scripts/cppcheck-suppressions.txt`.

Install the tools on Debian/Ubuntu:

```bash
sudo apt install clang-format clang-tidy cppcheck libeigen3-dev
```

On MSYS2 MinGW64: `pacman -S mingw-w64-x86_64-clang-tools-extra` (and cppcheck
from your preferred source).

## Tests

Unit tests build by default (`FX2D_BUILD_TESTS=ON`) into a single `Fx2DTests`
binary registered with CTest:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j --target Fx2DTests
ctest --test-dir build --output-on-failure
```

The suite covers the AABB tree, joints and motor control, CCD, capsule and
edge collisions, angle precision, resting-contact stability, and contact
events and sensors (`tests/test_*.cpp`). New solver or collision work should
land with a matching regression test.

### Writing a test

Each `tests/test_*.cpp` exposes one `run_*_tests()` entry point, registered in
the `kSuites` table in `tests/main.cpp`. Every suite runs even if an earlier one
fails, so one run reports every broken area rather than only the first.

A suite can be marked `slow` in that table. Slow suites are skipped when
`FX2D_SKIP_SLOW_TESTS=1`, and the skip is printed rather than silent. Only
`adversarial` is slow today: it simulates tall stacks for hundreds of steps,
which is seconds in Release but minutes under ASan/UBSan. CI sets the variable
for its Debug job alone, so those scenes still run on every push in Release.

Assert with `require()` / `require_near()` from `tests/test_harness.h`, never
with `assert()` from `<cassert>`. Release builds define `NDEBUG`, which expands
`assert()` to nothing — an assert-based test silently passes without checking
anything. CI fails the build if `assert(` reappears under `tests/`.

### Changing the solver without changing physics

`tests/test_solver_regression.cpp` is the net for refactors meant to change speed and nothing
else. Every other suite asserts a physical property — a stack stands, energy does not rise —
and those stay true under a solver that quietly drifts. This one pins the numbers themselves,
in two parts:

- **Repeatability.** The same scene built and stepped twice in one process must produce
  bit-identical state. Exact equality on purpose: what it catches is iteration order leaking
  into results, which every tolerance-based test in the suite would pass.
- **Golden state.** Four scenes stepped a fixed number of times, compared against
  `tests/goldens_solver_regression.inc` with tight but not bitwise tolerances, so a different
  compiler or ISA does not fail the suite on its last bits.

If a change is supposed to be speed-only and this suite fails, the change altered physics.
Regenerate the table only when the change to physics was deliberate:

```bash
FX2D_PRINT_GOLDENS=1 ./build/Fx2DTests   # prints a paste-ready table instead of asserting
```

## House rules

Two rules that apply to every change, and that reviews should reject on.

**1. No comment block longer than three lines.** If the explanation needs more, it belongs in
`docs/` and the comment should point there. Long rationale blocks in headers go stale faster than
the code, and they push the code they describe off the screen. Check with:

```bash
python - <<'EOF'
import io, os
for root in ('include/Fx2D', 'src', 'tests', 'scripts', 'examples'):
    for d, _, fs in os.walk(root):
        for f in sorted(fs):
            if not f.endswith(('.h', '.cpp', '.inc')): continue
            p = os.path.join(d, f)
            lines = io.open(p, encoding='utf-8', errors='replace').read().replace(chr(13), '').split(chr(10))
            start, run = None, 0
            for i, line in enumerate(lines):
                if line.strip().startswith('//'):
                    start = i if start is None else start
                    run += 1
                else:
                    if run > 3: print('%s:%d (%d lines)' % (p, start + 1, run))
                    start, run = None, 0
EOF
```

**2. Reuse; never duplicate a function or method.** If two pieces of code do the same job, one of
them must go — extract the shared part, or delete the loser. This has bitten twice already: a
scalar velocity kernel left behind after the batched one replaced it, and a `project_onto` that
`min_projection` had superseded. Both were dead *and* duplicated, which is the worst of both.
Before adding a helper, grep for one that already does the job.

**3. Delete stale and unused code.** Not "comment it out", not "leave it, someone might want
it" — delete it. Dead code is read as if it were live, and it hides worse things: the scalar
velocity kernel that lingered after the batched one replaced it was both dead *and* a duplicate,
and an accessor that existed but was never wired up (`is_overflow_group`) turned out to be a
latent correctness gap rather than mere clutter. Public library API that users may call is not
stale; internal helpers with no caller are.

**4. Minimal, modular, efficient, readable — in that order of doubt.** Prefer the smaller
change; prefer the version with fewer moving parts; and when efficiency and readability pull
apart, say why in a comment and point at the measurement. Anything clever enough to need
defending needs a number behind it — see *Measure in cycles* below.

## Benchmarking

Performance claims need numbers. The step benchmark is a separate target, off by default:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DFX2D_BUILD_BENCH=ON
cmake --build build -j --target Fx2DBench
./build/Fx2DBench                 # all scenes, 300 steps per size
./build/Fx2DBench 500 pile        # one scene, 500 steps per size
```

Three scenes, because body count does not predict solver load:

| scene | shape | what it stresses |
|---|---|---|
| `settling_boxes` | a loose grid of boxes falling onto ground, 10–3000 bodies | per-body work; sparse contacts |
| `pile` | circles packed into a walled container, 50–1200 bodies | the contact-dense case — 1200 bodies produce ~3500 contacts |
| `stacks` | free-standing 10-box columns | solver convergence over deep contact chains |

Each row reports wall time, CPU time, their ratio, and the **mean live contact count**. Report
wall and CPU **both**: a change that halves wall time while spending eight times the CPU is
usually the wrong trade for a library that may be one subsystem among many, and a bad trade for
RL rollouts where many independent sims already saturate the machine. A `cpu/wall` ratio near
1.0 means serial; well above 1.0 is only a win if wall time actually fell. Any threading change
needs a before/after from this target — see item 7 under Priority Targets in
[docs/ToDo.md](ToDo.md).

### Reading the phase split

`-DFX2D_PROFILE=ON` compiles in the per-phase timers (`include/Fx2D/Profile.h`) and the
benchmark prints where the step actually goes — broad phase, narrow phase, position solve,
constraints, integration, velocity derivation, velocity passes, bookkeeping, and the step
total. Off by default; the timers are cheap but not free.

```bash
cmake -S . -B build-prof -DCMAKE_BUILD_TYPE=Release -DFX2D_BUILD_BENCH=ON -DFX2D_PROFILE=ON
cmake --build build-prof -j --target Fx2DBench
./build-prof/Fx2DBench 150 pile
```

Use it to decide *what* to optimise before optimising it. The rankings in
[docs/simd_plan.md](simd_plan.md) have been overturned twice by re-running this.

### Look for allocator traffic first

`FX2D_PROFILE` says which phase is expensive; it does not say why. Before optimising the
arithmetic in a phase, take a sampling profile and look for `operator new`, `posix_memalign`
and `cfree`:

```bash
sudo apt install linux-tools-generic
perf record -g --call-graph=dwarf -F 300 ./build/Fx2DBench 60 stacks
perf report --stdio --no-children -g none | head -20
```

This has now paid repeatedly, and always the same way: **an accessor or expression that returns
by value, called in a loop, read through a `const auto&` that makes the copy invisible.**

Each of these is how the code **used to** read; all four have since been fixed, and they are
kept here because the shape of the mistake is what recurs:

```cpp
const auto& v = shape->vertices();       // copied every vertex into a fresh aligned allocation
const auto& bb = entity.bounding_box();  // same
auto g = entity->collision_geometry();   // an atomic increment and decrement
project_onto(axis, origin).argmin()      // two temporary FxArrays per call
```

Binding a reference to a temporary extends its lifetime -- it does not avoid creating it. Every
one of those looked like a free read at the call site. Together they were 11.5% of the `stacks`
step in the allocator plus 3.3% of the `pile` step in `shared_ptr` refcounting, and fixing them
was worth 1.15-1.6x with bit-identical results. No amount of vectorising the surrounding maths
would have found any of it.

So: when adding an accessor to a type that appears in the step, return by reference unless the
caller genuinely needs a copy -- and when optimising, grep the hot path for `auto&` bound to a
call before touching the arithmetic.

### Measure in cycles, not wall time

On a laptop that moves on and off turbo, wall time is not a measurement. During one session the
*same unchanged binary* reported `settling_boxes/200` at 8.50, 11.89 and 3.47 ms/step in three
different windows -- a 3.4x spread on identical code. Two separate changes were nearly reported
as large wins on the strength of readings like that, and one of them was actually a regression.

Cycle counts are frequency-independent: the same code costs the same count whatever the clock is
doing. Use them for anything under about 20%:

```bash
perf stat -x, -e cycles,instructions -r 3 ./build/Fx2DBench 40 stacks
```

Rules that follow from this:

- **Interleave, never batch.** Run A, B, A, B... Running all of A's repeats then all of B's lets
  a slow window land entirely on one binary. Taking the minimum of several runs helps too, since
  contention only ever adds time.
- **Compare instruction counts alongside cycles.** If instructions are unchanged and cycles moved,
  the change was memory or scheduling; if both moved, it was work.
- **Check contact counts.** The benchmark prints them. A change that "sped things up" while
  producing fewer contacts may be simulating less, not running faster.
- **Rebuild what you are measuring.** More than one confusing result in this project has been a
  stale `Fx2DBench` -- `cmake --build ... --target Fx2DTests` does not rebuild the benchmark.

### Re-test rejected ideas when the ground moves

Full-step swept broad-phase proxies were implemented, measured, and reverted for costing more
than they saved. Re-tried later -- after unrelated work had made the narrow phase roughly three
times cheaper -- the same change was worth 1.4-1.6x on `settling_boxes` and 1.8-2.2x on `pile`.
Nothing about the idea changed; what changed was everything around it. A rejection is a
measurement of a moment, so record the reason alongside the result, and revisit it when that
reason stops holding.

### Measuring honestly

- **A/B in one binary where you can.** Two separate builds compared across a long run pick up
  thermal and machine-state drift; a switch inside one binary, run interleaved and repeated,
  does not. Two runs that disagree about the sign of a change are noise, not a result.
- **Compare contact counts, not just time.** A change that "sped things up" while producing
  fewer contacts may have lost coverage, or may simply have settled the scene differently.
  The benchmark prints the count so the two are distinguishable.
- **FP contraction is pinned** by `FX2D_PIN_FP_CONTRACT` (default on), which passes
  `-ffp-contract=off` / `/fp:precise` and propagates to consumers, since the maths lives in
  headers. Without it, whether `a*b+c` fuses into an FMA is the compiler's choice and varies by
  target — so an A/B between two builds would be comparing two different sets of arithmetic.
  Leave it on unless you are deliberately measuring the cost of the pin itself.

### What CI runs

`.github/workflows/tests.yml` builds and runs the suite on every push and pull
request, in both Release and Debug (Debug enables ASan and UBSan), with
`-Werror`. It configures with `-DFX2D_HEADLESS=ON`, which drops the renderer so
the physics core and tests build without raylib or ImGui. You can reproduce it
locally with:

```bash
cmake -S . -B build-ci -DCMAKE_BUILD_TYPE=Debug -DFX2D_HEADLESS=ON -DFX2D_WERROR=ON
cmake --build build-ci -j --target Fx2DTests
ctest --test-dir build-ci --output-on-failure
```

Run the Release configuration too — it is the one that runs the slow adversarial suite.

CI covers six jobs: Linux GCC (Release, Debug/ASan, and Release with `FX2D_NATIVE=ON`), Linux
Clang, macOS on Apple silicon, Windows MSVC, and a Linux cross-compile to aarch64 run under
qemu. Between them every combination that matters is exercised — two ISAs, three compiler
families, vectorised and portable builds.

## Portability

Fx2D has no architecture-specific code, and the intent is to keep it that way: write loops the
compiler can vectorise on any target, and reach for intrinsics only where `-fopt-info-vec`
proves it failed to. What does need care is the build.

### Architecture flags

`CMAKE_SYSTEM_PROCESSOR` spells the same architecture four ways — `x86_64`, `AMD64`, `aarch64`,
`arm64` — so `CMakeLists.txt` normalises it into `FX2D_ARCH` and selects tuning flags from that.
Every architecture flag goes through `fx2d_add_flag_if_supported`, which probes the compiler
first, because the wrong flag is a hard error rather than a warning:

- `-march=native` **does not exist** on Apple Clang for arm64. The arm64 path asks for
  `-mcpu=native` instead, and probes even that, since older Apple Clang lacks it and a
  cross-compiler rejects it outright (it has no "native" to detect).
- `/arch:` **does not exist** for MSVC ARM64. NEON is mandatory in ARMv8, so there is nothing to
  request — the vector unit is already assumed.

Two knobs:

| option | default | meaning |
|---|---|---|
| `FX2D_NATIVE` | `ON` | Tune for the machine doing the build. Right for development, wrong for anything you ship. |
| `FX2D_ARCH_BASELINE` | `""` | A fixed ISA the binary may assume everywhere: `sse2`, `avx2`, `avx512`, `armv8-a`, `armv8.2-a`. Takes precedence over `FX2D_NATIVE`. |

### Determinism across targets

Results are reproducible **within** a build configuration, not across every possible one. Two
things make that hold as well as it does:

- **`FX2D_PIN_FP_CONTRACT`** (default on) passes `-ffp-contract=off` / `/fp:precise`. This is
  not theoretical: building for aarch64 with contraction left at the compiler default makes
  `tests/test_solver_regression.cpp` fail, because the fused multiply-add rounds differently
  than the separate multiply and add the goldens were captured from.
- **Tolerances, not bit patterns.** The golden tables are compared with absolute tolerances for
  exactly this reason. They currently pass on GCC x86-64, Clang x86-64, GCC aarch64 under qemu,
  and MinGW GCC with LTO — but do not tighten them to bitwise equality, because that would hold
  only on the machine that captured them.

**What is actually achieved, measured.** The rule the project *enforces* is bitwise
repeatability within one process — the regression suite's first half, exact equality, no
tolerance. Across builds it only promises tolerances.

In practice it currently does better than that. Printing solver state at `setprecision(9)`,
which round-trips a float exactly, the four golden scenes come out **byte-identical** on GCC
x86-64, Clang x86-64, and GCC aarch64 run under qemu. So does a deliberately harder scene: a
dozen boxes tumbling at 4-14 rad/s for 400 steps, which drives `set_world_pose` through
`std::sin`/`std::cos` at real angles and therefore through aarch64 glibc's own libm.

Treat that as an observed property, not a guarantee. It holds because FP contraction is pinned
off, nothing uses fast-math, and the arithmetic is plain add/sub/mul/div/sqrt, which IEEE-754
requires to be correctly rounded everywhere. It could legitimately break on a libm version whose
`sin` differs in the last bit, on real ARM hardware rather than qemu, or on MSVC — none of which
has been checked. Nothing in CI asserts it, and promising it would be a strong claim to defend.

If you are building on cross-machine reproducibility — lockstep networking, RL rollouts compared
across hosts — this is the thing to verify yourself on your own targets first, and to add a CI
check for if you depend on it.

### Testing ARM without ARM hardware

`cmake/toolchains/aarch64-linux-gnu.cmake` cross-compiles with the Debian/Ubuntu aarch64 GCC,
and qemu-user runs the result. The full suite passes this way, so an ARM claim can be checked
before it reaches CI:

```bash
sudo apt install g++-aarch64-linux-gnu qemu-user

# yaml-cpp has to exist for the target; Eigen is header-only so the host copy serves both.
git clone --depth 1 --branch 0.8.0 https://github.com/jbeder/yaml-cpp.git yaml-cpp-src
cmake -S yaml-cpp-src -B yaml-cpp-arm64 \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/toolchains/aarch64-linux-gnu.cmake \
  -DCMAKE_INSTALL_PREFIX=$HOME/arm64-sysroot \
  -DYAML_BUILD_SHARED_LIBS=OFF -DYAML_CPP_BUILD_TESTS=OFF -DYAML_CPP_BUILD_TOOLS=OFF
cmake --build yaml-cpp-arm64 -j && cmake --install yaml-cpp-arm64

cmake -S . -B build-arm64 -DCMAKE_BUILD_TYPE=Release -DFX2D_HEADLESS=ON \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/toolchains/aarch64-linux-gnu.cmake \
  -DCMAKE_PREFIX_PATH=$HOME/arm64-sysroot -DFX2D_WERROR=ON
cmake --build build-arm64 -j --target Fx2DTests
qemu-aarch64 -L /usr/aarch64-linux-gnu ./build-arm64/Fx2DTests
```

### The vectoriser budget

`CMakeLists.txt` passes `--param=vect-max-version-for-alias-checks=200` to GCC. This is not
tuning trivia: GCC abandons a loop needing more runtime alias checks than that budget allows
(default 10), and the batched contact solve touches around twenty columns. Without the flag both
hot loops in `resolve_velocities_batched` silently fall back to scalar and the `stacks` step is
about 26% slower -- **with no test failing**, because the results are identical either way.

If you are changing that kernel, check the loops still vectorise:

```bash
g++ -std=c++20 -O3 -march=native -I include -I /usr/include/eigen3 \
    -c src/Constraints.cpp -o /dev/null -fopt-info-vec | grep vectorized
```

Two things reliably break it, both learned the hard way: a `bool` temporary holding a comparison
(the vectoriser has no vector type for it and gives up on the whole loop -- keep the comparison
inline in the select), and letting the run mix manifold sizes.

The flag is gated on the compiler being GCC. Clang accepts the spelling and then warns that it
went unused, which `-Werror` turns into an error.

### Warning levels differ by compiler

Clang's `-Wconversion` implies `-Wsign-conversion`; GCC's does not. Building with Clang before
pushing is the cheapest way to catch that class, and the `clang` CI job is there to keep it from
accumulating between macOS runs. MSVC `/W4` flags a different set again, which is why
`FX2D_WERROR` is off for the MSVC job until those have been worked through.
