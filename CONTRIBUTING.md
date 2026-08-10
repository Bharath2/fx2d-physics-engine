# Contributing to Fx2D

Contributions are welcome! Please follow the existing code style and conventions.

- **Bugs & feature requests:** Open a new issue describing the problem or
  proposal clearly (steps to reproduce, or the rationale/use case).
- **Working on features/fixes:** Create a new branch from `main`, commit your
  changes, and open a pull request referencing the issue.
- Run `./scripts/format.sh` before opening a PR; `./scripts/lint.sh` should
  pass. Both are gated in CI on every push and pull request.
- **Looking for something to work on?** The roadmap of open targets lives in
  [docs/ToDo.md](./docs/ToDo.md) — each item records the motivation, the
  relevant code paths, and the suggested approach.

## Linting

Style and light static analysis (clang-format, cppcheck, clang-tidy) are gated
in CI (`.github/workflows/lint.yml`):

```bash
./scripts/format.sh           # rewrite sources to .clang-format
./scripts/format.sh --check   # dry-run (fails if anything would change)
./scripts/lint.sh             # format check + cppcheck + clang-tidy
./scripts/lint.sh --no-tidy   # skip clang-tidy (fastest local gate)
```

`lint.sh` generates a minimal `compile_commands.json` for clang-tidy via
`./scripts/gen_lint_compile_db.sh` (physics core only, no raylib), and cppcheck
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
edge collisions, angle precision, and resting-contact stability
(`tests/test_*.cpp`). New solver or collision work should land with a
matching regression test.
