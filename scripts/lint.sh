#!/usr/bin/env bash
# Lint gate: clang-format --check, cppcheck, clang-tidy.
# Flags: --no-tidy, --no-format
set -euo pipefail
cd "$(dirname "$0")/.."

DO_FORMAT=1
DO_TIDY=1
for arg in "$@"; do
  case "$arg" in
    --no-tidy) DO_TIDY=0 ;;
    --no-format) DO_FORMAT=0 ;;
    -h|--help)
      sed -n '2,4p' "$0"
      exit 0
      ;;
    *)
      echo "unknown option: $arg" >&2
      exit 2
      ;;
  esac
done

status=0

if [[ $DO_FORMAT -eq 1 ]]; then
  echo "==> clang-format"
  if ! ./scripts/format.sh --check; then
    echo "clang-format failed — run ./scripts/format.sh to fix" >&2
    status=1
  fi
fi

echo "==> cppcheck"
if ! command -v cppcheck >/dev/null 2>&1; then
  echo "error: cppcheck not found on PATH" >&2
  status=1
else
  # Treat warnings/errors as failures; style stays informational.
  if ! cppcheck \
      --enable=warning,performance,portability \
      --std=c++20 \
      --language=c++ \
      --inline-suppr \
      --suppressions-list=scripts/cppcheck-suppressions.txt \
      --error-exitcode=1 \
      --quiet \
      -I include \
      src tests examples; then
    echo "cppcheck reported issues" >&2
    status=1
  else
    echo "cppcheck: OK"
  fi
fi

if [[ $DO_TIDY -eq 1 ]]; then
  echo "==> clang-tidy"
  if ! command -v clang-tidy >/dev/null 2>&1; then
    echo "error: clang-tidy not found on PATH" >&2
    status=1
  else
    ./scripts/gen_lint_compile_db.sh build-lint
    # Physics + headless examples only — Renderer.cpp needs raylib headers.
    TIDY_FILES=(
      src/Entity.cpp
      src/Scene.cpp
      src/Collisions.cpp
      src/Constraints.cpp
      src/Joints.cpp
      src/YamlUtils.cpp
    )
    if ! clang-tidy -p build-lint --quiet "${TIDY_FILES[@]}"; then
      echo "clang-tidy reported issues" >&2
      status=1
    else
      echo "clang-tidy: OK"
    fi
  fi
fi

if [[ $status -ne 0 ]]; then
  echo "lint: FAILED" >&2
  exit "$status"
fi
echo "lint: all checks passed"
