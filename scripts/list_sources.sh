#!/usr/bin/env bash
# List first-party sources. Default: everything format/lint sweeps (excludes lib/, build*).
# --core: the renderer-free physics translation units, the single source of truth for every
#         consumer that compiles or lints the core (headless build, compile DB, clang-tidy).
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ "${1:-}" == "--core" ]]; then
  printf '%s
'     src/Entity.cpp     src/Profile.cpp     src/Scene.cpp     src/Collisions.cpp     src/ContactGraph.cpp     src/Constraints.cpp     src/Joints.cpp     src/Query.cpp     src/YamlUtils.cpp
  exit 0
fi

find include/Fx2D src tests examples scripts   \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \)   ! -path '*/lib/*'   ! -path '*/build*/*'   | sort
