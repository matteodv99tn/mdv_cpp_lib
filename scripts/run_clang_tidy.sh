#!/bin/bash
set -euo pipefail

# Simplification of the shell script in
# https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/run_clang_tidy.sh

# Diagnostics depend on the system headers (libstdc++, Eigen, CGAL, ...), so
# they are only authoritative on Arch Linux, the reference platform.
os_id=$(. /etc/os-release 2>/dev/null && printf '%s' "${ID:-}")
if [[ $os_id != arch ]]; then
  printf 'clang-tidy: skipped on "%s", only Arch Linux is supported\n' \
    "${os_id:-unknown}" >&2
  exit 0
fi

script_dir=${0%/*}
ws_dir=$(realpath "$script_dir/..")
build_dir="$ws_dir/build"
paths=("$ws_dir/src" "$ws_dir/include" "$ws_dir/unit_tests" "$ws_dir/executables")

if [[ ! -f "$build_dir/compile_commands.json" ]]; then
  printf 'Error: compile_commands.json not found in %s\n' "$build_dir"
  printf 'Run: cmake -B build -DBUILD_TESTING=ON -DBUILD_EXECUTABLES=ON -DUSE_RERUN=ON\n'
  exit 1
fi

llvm_dir=$("$script_dir/get_llvm.sh")

# Generated data tables are not worth linting
excluded=("$ws_dir/executables/dmp/data.cpp")
exclude_args=()
for f in "${excluded[@]}"; do
  exclude_args+=(! -path "$f")
done

find "${paths[@]}" -type f \
  \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) \
  ! -name '*_WIN.cpp' "${exclude_args[@]}" -print0 |
  python3 "$script_dir/lint_cache.py" "$build_dir" "$llvm_dir/bin/clangd"
