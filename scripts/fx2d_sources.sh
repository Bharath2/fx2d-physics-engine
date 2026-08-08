#!/usr/bin/env bash
# List first-party sources for format/lint (excludes lib/ and build*).
set -euo pipefail
cd "$(dirname "$0")/.."

find include/Fx2D src tests examples \
  \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) \
  ! -path '*/lib/*' \
  ! -path '*/build*/*' \
  | sort
