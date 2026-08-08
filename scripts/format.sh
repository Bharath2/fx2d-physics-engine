#!/usr/bin/env bash
# Format Fx2D first-party sources with clang-format, or check them.
#
#   ./scripts/format.sh          # rewrite in place
#   ./scripts/format.sh --check  # exit non-zero if any file would change
set -euo pipefail
cd "$(dirname "$0")/.."

if ! command -v clang-format >/dev/null 2>&1; then
  echo "error: clang-format not found on PATH" >&2
  exit 1
fi

mapfile -t FILES < <(./scripts/fx2d_sources.sh)
if [[ ${#FILES[@]} -eq 0 ]]; then
  echo "error: no sources found" >&2
  exit 1
fi

if [[ "${1:-}" == "--check" ]]; then
  echo "Checking clang-format on ${#FILES[@]} files..."
  clang-format --dry-run --Werror "${FILES[@]}"
  echo "clang-format: OK"
else
  echo "Formatting ${#FILES[@]} files..."
  clang-format -i "${FILES[@]}"
  echo "clang-format: wrote changes"
fi
