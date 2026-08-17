#!/usr/bin/env bash
# clang-format rewrite; pass --check for dry-run (CI).
set -euo pipefail
cd "$(dirname "$0")/.."

STYLE="$(cd "$(dirname "$0")" && pwd)/.clang-format"

if ! command -v clang-format >/dev/null 2>&1; then
  echo "error: clang-format not found on PATH" >&2
  exit 1
fi

mapfile -t FILES < <(./scripts/list_sources.sh)
if [[ ${#FILES[@]} -eq 0 ]]; then
  echo "error: no sources found" >&2
  exit 1
fi

if [[ "${1:-}" == "--check" ]]; then
  echo "Checking clang-format on ${#FILES[@]} files..."
  clang-format "--style=file:$STYLE" --dry-run --Werror "${FILES[@]}"
  echo "clang-format: OK"
else
  echo "Formatting ${#FILES[@]} files..."
  clang-format "--style=file:$STYLE" -i "${FILES[@]}"
  echo "clang-format: wrote changes"
fi
