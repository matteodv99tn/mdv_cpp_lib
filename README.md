# Matteo Dalle Vedove's CPP library

This repository contains my personal C++ library for my research work and activities.

Within this library, I also host some documentation and blog-post-like content that can be accessed [here](https://matteodv99tn.github.io/mdv_cpp_lib/index.html).


## Dependency installation

Some dependencies, required to build the library, can be installed using the package manager of your distribution.
Currently, only arch and ubuntu-based distributions are supported.
To install the dependencies on your machine, you may call the [`install_dependencies.sh`](./cmake/install_dependencies.sh) script:
```
./cmake/install_dependencies.sh
```
On ubuntu system, this is equivalent to updating the APT package list and installing the following packages:
```
apt-get install -y \
        cmake build-essential \
        libeigen3-dev libfmt-dev \
        libspdlog-dev libcgal-dev libgtest-dev \
        librange-v3-dev libmsgsl-dev libmatio-dev
```

The lonely (optional) dependency which cannot be installed by APT is the [rerun](https://github.com/rerun-io/rerun) C++ SDK.
For convenience, the [matteodv99tn/rerun-sdk-precompiled](https://github.com/matteodv99tn/rerun-sdk-precompiled) repository hosts the precompiled binaries of the SDK as a [prerelease](https://github.com/matteodv99tn/rerun-sdk-precompiled/releases/tag/prerelease).

On Ubuntu 22.04, you may install the precompiled debian package by first fetching it
```bash
curl -LO https://github.com/matteodv99tn/rerun-sdk-precompiled/releases/download/prerelease/rerun-cpp-sdk_0.20_3.deb
```
and then installing it
```bash
dpkg -i rerun-cpp-sdk*.deb
```


**Note:** this is tested to work only on Ubuntu 22.04 machine. Installing this package on other system may not work, due to ABI breaking of the arrow library which is bundled with rerun.
For instance, installing on Arch linux may couse problems.

### Build from source

If you want to build the external dependencies from source, a convenient [``CMakeLists.txt``](cmake/CMakeLists.txt) file is provided in order to preliminary fetch those library using cmake's [``ExternalProject``](https://cmake.org/cmake/help/latest/module/ExternalProject.html) functionalities.

- Configure the project
  ```bash
  cmake -S cmake -B cmake/build
  ```
- Build the external dependencies
  ```bash
  cmake --build cmake/build
  ```

These libraries will be installed at `cmake/install`, and such path will be automatically added by `cmake` to its path within the main [`CMakeLists.txt`](./CMakeLists.txt) of the library.


### Rosdep

This library can also be built as a ROS2 package.
This means that you may install all external dependencies using [rosdep](https://wiki.ros.org/rosdep).
On a machine with ROS2 installed, you may simply:

0. If you don't have `rosdep` on the machine, you can install it through `apt` and then initialise it:
   ```bash
   sudo apt-get update && sudo apt-get install -y python3-rosdep
   sudo rosdep init
   ```
   **Note:** the `sudo rosdep init` shall be called only once after the installation.
1. update the `rosdep` package list:
   ```
   rosdep update
   ```
1. install the dependencies
   ```
   rosdep install --from-paths <path/to/mdvcpplib> --ignore-src -r -y
   ```

## Development environment setup

### Linting and formatting

This library uses pinned LLVM 22.1.8 tools for formatting and static analysis.

- `./scripts/run_clang_format.sh` formats all library source files, or only the filenames supplied as arguments.
- `./scripts/run_clang_tidy.sh` uses `clangd --clang-tidy` to check sources and headers in `src`, `include`, `unit_tests`, and `executables`, using `build/compile_commands.json`. Configure and build dependencies first as shown below.

> [!NOTE]
> `scripts/get_llvm.sh` supports Linux `x86_64` and `aarch64` and requires `curl`, `tar` with xz support, `sha256sum`, and `flock`.
> Both quality scripts automatically bootstrap the same tools. The first run downloads the LLVM release archive (about 1.9 GiB on x86_64), verifies its SHA-256 checksum, and extracts only `clangd`, `clang-format`, and Clang resource headers. The archive is deleted after successful installation.
> The extracted tools are cached in `$XDG_CACHE_HOME/llvm` (or `$HOME/.cache/llvm`). `get_llvm.sh` prints the installation directory; concurrent installations share a lock.
> Set the `MDVCPPLIB_LLVM_CACHE_DIR` environment variable to specify the caching location.

### Pre-commit hooks

The `pre-commit` hook formats staged C and C++ files using the same pinned LLVM tools as the scripts.
The `pre-push` hooks run static analysis and a clean build and test run in a temporary directory.

Install pre-commit and both hook stages:

```bash
python3 -m pip install pre-commit
pre-commit install
```

Run formatting manually:

```bash
pre-commit run --all-files --hook-stage pre-commit
```

Install the Rerun SDK (see dependency installation above), then configure `build` with tests and executables enabled and build the external dependency before running static analysis. Repeat configuration when targets or build options change. Run both pre-push checks manually:

```bash
cmake -B build -DBUILD_TESTING=ON -DBUILD_EXECUTABLES=ON -DUSE_RERUN=ON
cmake --build build --target matioCpp --parallel
pre-commit run --all-files --hook-stage pre-push
```

The clean build can also be run independently with `./scripts/run_clean_build.sh`.
Every push builds the library and Rerun-backed executables and runs the unit tests in a clean build, which can take some time. The interactive demo executables are build-checked, not launched by CTest.


### Python bindings (developer workflow)

You can build and install python bindings through `pip`. From the repository root:

```
uv venv .venv -p 3.12
uv pip install -e .
uv pip install pytest
```

#### Development

`pip install -e .` triggers a full C++ build (expensive). For iterative binding
development, run it once to set up the environment, then use a bindings-only CMake
build and install directly into your active `site-packages`.

Configure (from repo root):

```
CMAKE_INSTALL_MODE=ABS_SYMLINK \
  cmake -B build \
  -DBUILD_BINDINGS=ON \
  -DCMAKE_INSTALL_PREFIX=$(python -c "import sysconfig; print(sysconfig.get_paths()['purelib'])")
```

Build:

```
CMAKE_INSTALL_MODE=ABS_SYMLINK cmake --build build --parallel
```

Install:

```
CMAKE_INSTALL_MODE=ABS_SYMLINK cmake --install build
```

Run the bindings tests:

```
uv run pytest bindings/tests
```

**Note**: setting [`CMAKE_INSTALL_MODE=ABS_SYMLINK`](https://cmake.org/cmake/help/latest/envvar/CMAKE_INSTALL_MODE.html)
creates symlink installs. This lets you edit the wrapper `.py` files directly
from `site-packages` while still tracking changes in git.


### Local testing of changes

This library provides some minimal CI/CD checks using [Github Actions](https://github.com/features/actions) to ensure that the code is always in a working state.

If you want to make sure that all the changes you are making are not breaking all possible build configuration and tests, you may try to run the github actions locally using [act](https://github.com/nektos/act).

After installing _act_ (you can find the installation instructions [here](https://nektosact.com/installation/)), first:

- create a path for the artifacts to be stored, as they are required by the actions themselves.
  You can do this by running:
  ```
  mkdir -p /tmp/artifacts
  ```
  Feel free to change the path to your preferred one.
- Then, you may run _act_ at the root of the directory with the following command:
  ```
  act --env ACTIONS_RUNTIME_URL=http://artifacts.docker.internal:8080/ \
      --env ACTIONS_RUNTIME_TOKEN=foo \
      --env ACTIONS_CACHE_URL=http://artifacts.docker.internal:8080/ \
      --artifact-server-path /tmp/artifacts
  ```
  The extra arguments are required to prorperly setup your local machine to mock the github artifacts server.

## Credits

- [pbdlib-matlab](https://gitlab.idiap.ch/rli/pbdlib-matlab) for the handwritten letter dataset;
