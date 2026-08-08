#!/usr/bin/env bash
# Minimal compile_commands.json for clang-tidy (physics core, no raylib).
set -euo pipefail
cd "$(dirname "$0")/.."

OUT_DIR=${1:-build-lint}
mkdir -p "$OUT_DIR"

CXX=${CXX:-clang++}
STD=${STD:-c++20}

to_mixed() {
  # Prefer a drive-letter path clang understands on Windows/MSYS.
  if command -v cygpath >/dev/null 2>&1; then
    cygpath -m "$1"
  else
    echo "$1"
  fi
}

# Prefer known Eigen install prefixes.
EIGEN_INC=${EIGEN_INC:-}
if [[ -z "$EIGEN_INC" ]]; then
  for cand in /usr/include/eigen3 /usr/local/include/eigen3 \
              /mingw64/include/eigen3 /ucrt64/include/eigen3 \
              "C:/msys64/mingw64/include/eigen3" \
              "C:/msys64/ucrt64/include/eigen3"; do
    if [[ -d "$cand" ]]; then EIGEN_INC=$cand; break; fi
  done
fi
if [[ -z "$EIGEN_INC" ]]; then
  echo "error: Eigen3 headers not found; set EIGEN_INC" >&2
  exit 1
fi
EIGEN_INC=$(to_mixed "$EIGEN_INC")

# yaml-cpp is only required by YamlUtils.cpp; optional for the rest.
YAML_INC=${YAML_INC:-}
if [[ -z "$YAML_INC" ]]; then
  for cand in /usr/include /usr/local/include \
              /mingw64/include /ucrt64/include \
              "C:/msys64/mingw64/include" \
              "C:/msys64/ucrt64/include"; do
    if [[ -f "$cand/yaml-cpp/yaml.h" ]]; then YAML_INC=$cand; break; fi
  done
fi
YAML_FLAG=
if [[ -n "$YAML_INC" ]]; then
  YAML_INC=$(to_mixed "$YAML_INC")
  YAML_FLAG="-I$YAML_INC"
fi

ROOT=$(to_mixed "$(pwd)")

FILES=(
  src/Entity.cpp
  src/Scene.cpp
  src/Collisions.cpp
  src/Constraints.cpp
  src/Joints.cpp
  src/YamlUtils.cpp
)

{
  echo '['
  first=1
  for f in "${FILES[@]}"; do
    [[ -f "$f" ]] || continue
    cmd="$CXX -std=$STD -I$ROOT/include -I$EIGEN_INC ${YAML_FLAG} -c $ROOT/$f"
    if [[ $first -eq 0 ]]; then echo ','; fi
    first=0
    printf '  {\n'
    printf '    "directory": "%s",\n' "$ROOT"
    printf '    "command": "%s",\n' "$cmd"
    printf '    "file": "%s/%s"\n' "$ROOT" "$f"
    printf '  }'
  done
  echo
  echo ']'
} > "$OUT_DIR/compile_commands.json"

echo "Wrote $OUT_DIR/compile_commands.json (${#FILES[@]} translation units, Eigen=$EIGEN_INC)"
