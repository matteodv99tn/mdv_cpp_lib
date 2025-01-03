# Matteo Dalle Vedove's CPP library

This repository contains my personal C++ library for my research work and activities.

Within this library, I also host some documentation and blog-post-like content that can be accessed [here](https://matteodv99tn.github.io/mdv_cpp_lib/index.html).


## Dependency installation 

This library depends on external libraries to work, most of which cannot be intestalled through conventional package managers (``apt``, ``pacman``).

Still, they can be fetched using the [``CMakeLists.txt``](cmake/CMakeLists.txt) in the [``cmake``](./cmake) folder that internally uses cmake's [``ExternalProject``](https://cmake.org/cmake/help/latest/module/ExternalProject.html) functionalities. 

- Configure the project
  ``` bash
  cmake -S cmake -B cmake/build
  ```
- Build the external dependencies
  ``` bash
  cmake --build cmake/build
  ```
- Install the libraries
  ``` bash
  cmake --build cmake/build
  ```

These libraries will be installed at ``cmake/install``, and such path will be automatically added by ``cmake`` to its path.
