#!/bin/bash
set -euo pipefail

script_dir=${0%/*}
ws_dir=$(realpath "$script_dir/..")

llvm_dir=$("$script_dir/get_llvm.sh")

if (( $# )); then
  exec "$llvm_dir/bin/clang-format" -i -- "$@"
fi

find "${ws_dir}/include" "${ws_dir}/src" "${ws_dir}/unit_tests" "${ws_dir}/executables" \
    -type f \
    \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) \
    -print0 \
    | xargs -0 -r -n 1 -P "$(nproc)" "$llvm_dir/bin/clang-format" -i
