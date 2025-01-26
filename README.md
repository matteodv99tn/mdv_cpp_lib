# Matteo Dalle Vedove's CPP library

This repository contains my personal C++ library for my research work and activities.

Within this library, I also host some documentation and blog-post-like content that can be accessed [here](https://matteodv99tn.github.io/mdv_cpp_lib/index.html).


## Dependency installation 

Some dependencies, required to build the library, can be installed using the package manager of your distribution. 
Currently, only arch and ubuntu-based distributions are supported.
To install the dependencies on your machine, you may call the [`install_dependencies.sh`](./cmake/install_dependencies.sh) script:
``` bash
./cmake/install_dependencies.sh
```

Still, some library cannot be installed throug package managers (``apt``, ``pacman``); 
for this reason, a convienient [``CMakeLists.txt``](cmake/CMakeLists.txt) file is provided in order to preliminary fetch those library using cmake's [``ExternalProject``](https://cmake.org/cmake/help/latest/module/ExternalProject.html) functionalities. 

- Configure the project
  ``` bash
  cmake -S cmake -B cmake/build
  ```
- Build the external dependencies
  ``` bash
  cmake --build cmake/build
  ```

These libraries will be installed at `cmake/install`, and such path will be automatically added by `cmake` to its path within the main [`CMakeLists.txt`](./CMakeLists.txt) of the library.


## Local testing of changes

This library provides some minimal CI/CD checks using [Github Actions](https://github.com/features/actions) to ensure that the code is always in a working state. 

If you want to make sure that all the changes you are making are not breaking all possible build configuration and tests, you may try to run the github actions locally using [act](https://github.com/nektos/act).

After installing _act_ (you can find the installation instructions [here](https://nektosact.com/installation/)), first:

- create a path for the artifacts to be stored, as they are required by the actions themselves. 
  You can do this by running:
  ``` bash
  mkdir -p /tmp/artifacts
  ```
  Feel free to change the path to your preferred one.
- Then, you may run _act_ at the root of the directory with the following command:
  ``` bash
  act --env ACTIONS_RUNTIME_URL=http://artifacts.docker.internal:8080/ \
      --env ACTIONS_RUNTIME_TOKEN=foo \
      --env ACTIONS_CACHE_URL=http://artifacts.docker.internal:8080/ \
      --artifact-server-path /tmp/artifacts
  ```
  The extra arguments are required to prorperly setup your local machine to mock the github artifacts server.


