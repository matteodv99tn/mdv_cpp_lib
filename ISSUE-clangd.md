# Deferred: clangd warning gate and slow-check coverage

Status: documented for future work; the changes below are not enabled.

`scripts/run_clang_tidy.sh` intentionally follows dlolib's `clangd --check`
workflow. Repository-specific wiring covers `src`, `include`, `unit_tests`, and
`executables`, with compiler flags from `build/compile_commands.json`.

## Limitations

- In LLVM 22.1.8, `clangd --check` counts error-severity diagnostics for its exit
  status. Ordinary clang-tidy warnings do not make the command fail.
- Warnings are hidden by default. `--check-warnings` prints them, but does not
  change their severity or make them count as errors.
- Clangd's default `Diagnostics.ClangTidy.FastCheckFilter: Strict` excludes slow
  checks and checks whose speed is unknown, even if `.clang-tidy` enables them.
  Disabling location checks with `--check-locations=false` does not change this.

## Future fix while retaining clangd

To stop filtering out slow or unclassified checks, add the following project
`.clangd` configuration:

```yaml
Diagnostics:
  ClangTidy:
    FastCheckFilter: None
```

Ensure clangd configuration loading is enabled (`--enable-config`), and add
`--check-warnings` to make warning diagnostics visible. Neither setting makes
warnings fail the command. A warning-to-failure policy still needs to be
implemented and verified independently; do not mistake printed warnings for
a functioning lint gate. Disabling the speed filter also does not guarantee
that clangd supports every standalone clang-tidy check.

## Alternative: standalone clang-tidy

The previously proposed replacement avoids clangd's speed filter by running
`clang-tidy` directly against configured translation units:

```bash
python3 - "$build_dir/compile_commands.json" <<'PY' |
import json
import sys

with open(sys.argv[1]) as database:
    for filename in sorted({entry["file"] for entry in json.load(database)}):
        sys.stdout.buffer.write(filename.encode() + b"\0")
PY
  xargs -0 -r -n 1 -P "$(nproc)" "$llvm_dir/bin/clang-tidy" \
    -p "$build_dir" --warnings-as-errors='*'
```

This requires extracting `bin/clang-tidy` in `get_llvm.sh` and updating its
cache-hit check. Headers are analyzed through their including translation
units, subject to `.clang-tidy`'s header filter; unreferenced headers need
separate coverage. Configure tests and Rerun executables and build dependencies
before analysis so the compilation database and required headers are available.

Before adopting either approach, verify a clean fixture passes, a tidy-only
warning fails the intended gate, and a known slow check is actually executed.
No repository-wide formatting or lint fixes are part of this deferred work.

## References

- [LLVM 22.1.8 check-mode implementation](https://github.com/llvm/llvm-project/blob/llvmorg-22.1.8/clang-tools-extra/clangd/tool/Check.cpp)
- [Clangd FastCheckFilter configuration](https://clangd.llvm.org/config#fastcheckfilter)
- [Clang-tidy warnings-as-errors](https://clang.llvm.org/extra/clang-tidy/)
