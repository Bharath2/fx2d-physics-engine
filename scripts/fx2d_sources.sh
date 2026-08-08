#!/usr/bin/env bash
# Echo the Fx2D first-party sources that participate in format/lint.
# Excludes vendored code under lib/ and generated build trees.
set -euo pipefail
cd "$(dirname "$0")/.."

find include/Fx2D src tests examples \
  \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) \
  ! -path '*/lib/*' \
  ! -path '*/build*/*' \
  | sort
