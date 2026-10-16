#!/bin/bash
set -euo pipefail

llvm_version=22.1.8
cache_dir="${MDVCPPLIB_LLVM_CACHE_DIR:-${XDG_CACHE_HOME:-$HOME/.cache}/llvm}"

if [[ $(uname -s) != Linux ]]; then
  printf 'Error: only Linux is supported\n' >&2
  exit 1
fi

case $(uname -m) in
  x86_64)
    archive_name="LLVM-$llvm_version-Linux-X64.tar.xz"
    archive_sha256=df0e1ecf16caf3489a272a5eea4eec9b0d82878f6477fa309504f918a0006384
    ;;
  aarch64)
    archive_name="LLVM-$llvm_version-Linux-ARM64.tar.xz"
    archive_sha256=805efad2bb91cb4967fa569e0881d10c0f69c04461cf671cccbae19f547acc34
    ;;
  *)
    printf 'Error: unsupported Linux architecture: %s\n' "$(uname -m)" >&2
    exit 1
    ;;
esac

mkdir -p "$cache_dir"
cache_dir=$(realpath "$cache_dir")
llvm_dir="$cache_dir/${archive_name%.tar.xz}"
# Serialize downloads and publication, including the cache-hit check.
exec 9>"$cache_dir/.install.lock"
flock 9
if [[ ! -x $llvm_dir/bin/clangd || ! -x $llvm_dir/bin/clang-format ||
      ! -d $llvm_dir/lib/clang/${llvm_version%%.*}/include ]]; then
  archive="$cache_dir/$archive_name"
  archive_tmp="$archive.tmp"
  extract_dir=$(mktemp -d "$cache_dir/.llvm.XXXXXX")
  trap 'rm -f "$archive_tmp"; rm -rf "$extract_dir"' EXIT

  if [[ ! -f $archive ]]; then
    printf 'Downloading LLVM %s...\n' "$llvm_version" >&2
    curl --fail --location --retry 3 --output "$archive_tmp" \
      "https://github.com/llvm/llvm-project/releases/download/llvmorg-$llvm_version/$archive_name"
    printf '%s  %s\n' "$archive_sha256" "$archive_tmp" | sha256sum --check --status
    mv "$archive_tmp" "$archive"
  fi

  printf '%s  %s\n' "$archive_sha256" "$archive" | sha256sum --check --status
  tar --extract --file "$archive" --directory "$extract_dir" --wildcards \
    '*/bin/clangd' '*/bin/clang-format' '*/lib/clang/*/include/*'
  # Recover an incomplete installation without nesting one LLVM tree in another.
  rm -rf "$llvm_dir"
  mv -T "$extract_dir/${archive_name%.tar.xz}" "$llvm_dir"
  rm -f "$archive"
fi

printf '%s\n' "$llvm_dir"
