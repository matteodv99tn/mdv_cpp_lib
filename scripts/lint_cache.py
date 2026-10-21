#!/usr/bin/env python3
"""Run `clangd --check` on many files in parallel, caching the result of each file.

usage: lint_cache.py BUILD_DIR CLANGD < NUL-separated-files

The result of a file only depends on the clangd binary, .clang-tidy/.clangd, the compile
flags and the content of the files it includes, so unchanged files are replayed from the cache.
Files whose dependency scan fails (e.g. a missing header) are never cached.

Environment:
  MDV_LINT_JOBS       number of parallel workers (default 4)
  MDV_LINT_NO_CACHE   set to 1 to disable the cache
  MDV_LINT_CACHE_DIR  cache location (default $XDG_CACHE_HOME/mdv-lint or ~/.cache/mdv-lint)
"""
import hashlib
import json
import os
import shlex
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

MAX_AGE_S = 14 * 24 * 3600
DEP_FLAGS_WITH_ARG = {"-MT", "-MQ", "-MF"}
DEP_FLAGS = {"-MD", "-MMD"}

build_dir, clangd = sys.argv[1], sys.argv[2]
build_dir = os.path.abspath(build_dir)
ws = os.path.dirname(build_dir)
jobs = int(os.environ.get("MDV_LINT_JOBS", "4"))
use_cache = os.environ.get("MDV_LINT_NO_CACHE") != "1"
cache_dir = os.environ.get(
    "MDV_LINT_CACHE_DIR",
    os.path.join(os.environ.get("XDG_CACHE_HOME") or os.path.expanduser("~/.cache"), "mdv-lint"),
)
files = [f for f in sys.stdin.buffer.read().decode().split("\0") if f]


def scan_flags(entry):
    """Compiler and flags of a compile_commands entry, reduced to what affects preprocessing."""
    args = shlex.split(entry["command"]) if "command" in entry else entry["arguments"]
    flags, skip, source = [], False, os.path.join(entry["directory"], entry["file"])
    for a in args[1:]:
        if skip:
            skip = False
        elif a == "-o" or a in DEP_FLAGS_WITH_ARG:
            skip = True
        elif a == "-c" or a in DEP_FLAGS or a.startswith("-fdiagnostics-color"):
            pass
        elif os.path.join(entry["directory"], a) == source:
            pass
        else:
            flags.append(a)
    return args[0], flags


if use_cache:
    os.makedirs(cache_dir, exist_ok=True)
    with open(os.path.join(build_dir, "compile_commands.json")) as f:
        entries = json.load(f)
    by_file = {os.path.join(e["directory"], e["file"]): e for e in entries}
    # Headers are not in the database: borrow the flags of the first project source file.
    fallback = next(e for p, e in by_file.items() if p.startswith(ws + "/") and not p.startswith(build_dir.rstrip("/") + "/"))
    static = hashlib.sha256()
    for name in (".clang-tidy", ".clangd"):
        p = os.path.join(ws, name)
        if os.path.exists(p):
            static.update(open(p, "rb").read())
    static.update(subprocess.run([clangd, "--version"], capture_output=True, check=True).stdout)

    content_hashes = {}

    def content_hash(path):
        if path not in content_hashes:
            content_hashes[path] = hashlib.sha256(open(path, "rb").read()).hexdigest()
        return content_hashes[path]

    def key_for(path):
        compiler, flags = scan_flags(by_file.get(path, fallback))
        r = subprocess.run([compiler, *flags, "-x", "c++", "-M", "-w", path], capture_output=True, text=True)
        if r.returncode:
            return None
        h = static.copy()
        h.update(" ".join(flags).encode())
        for dep in sorted(set(r.stdout.replace("\\\n", " ").split()[1:])):
            if dep.startswith(ws + "/") and not dep.startswith(build_dir.rstrip("/") + "/"):
                h.update(f"{dep}:{content_hash(dep)}\n".encode())
            else:
                h.update(f"{dep}:{os.path.getsize(dep)}\n".encode())
        return h.hexdigest()


def run_clangd(path):
    r = subprocess.run(
        [clangd, "--log=error", "--clang-tidy", f"--compile-commands-dir={build_dir}",
         "--check-locations=false", f"--check={path}"],
        capture_output=True, text=True)
    return r.returncode, "".join(f"{path}: {l}\n" for l in (r.stdout + r.stderr).splitlines())


def lint(path_key):
    path, key = path_key
    entry = os.path.join(cache_dir, key) if key else None
    if entry and os.path.exists(entry):
        os.utime(entry)
        code, _, out = open(entry).read().partition("\n")
        return int(code), out, True
    code, out = run_clangd(path)
    if entry:
        with open(entry + ".tmp", "w") as f:
            f.write(f"{code}\n{out}")
        os.replace(entry + ".tmp", entry)
    return code, out, False


start = time.time()
keys = [None] * len(files)
if use_cache:
    # The dependency scan is cheap in memory, so it is not limited by MDV_LINT_JOBS.
    with ThreadPoolExecutor(os.cpu_count()) as pool:
        keys = list(pool.map(key_for, files))
with ThreadPoolExecutor(jobs) as pool:
    results = sorted(zip(files, pool.map(lint, zip(files, keys))))
for _, (_, out, _) in results:
    sys.stdout.write(out)
failed = sum(1 for _, (code, _, _) in results if code)
if use_cache:
    hits = sum(1 for _, (_, _, hit) in results if hit)
    print(f"lint cache: {hits}/{len(files)} files reused ({time.time() - start:.1f}s)", file=sys.stderr)
    for name in os.listdir(cache_dir):
        p = os.path.join(cache_dir, name)
        if time.time() - os.path.getmtime(p) > MAX_AGE_S:
            os.remove(p)
sys.exit(123 if failed else 0)
