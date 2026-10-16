#!/bin/bash
set -euo pipefail

script_dir=${0%/*}
ws_dir=$(realpath "$script_dir/..")
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT

cmake -S "$ws_dir" -B "$build_dir" \
  -DBUILD_TESTING=ON -DBUILD_EXECUTABLES=ON -DUSE_RERUN=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" --parallel
ctest --test-dir "$build_dir" --output-on-failure --no-tests=error
