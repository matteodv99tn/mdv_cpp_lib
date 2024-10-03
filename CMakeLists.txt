cmake_minimum_required(VERSION 3.10)

project(mesh_manifolds)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)


# Fetch pre-compiled open3d library
include(cmake/open3d.cmake)
include(cmake/range-v3.cmake) 
include(cmake/autodiff.cmake)
include(cmake/nlohmann_json.cmake) 
include(cmake/matplotlibcpp.cmake) 

include(FetchContent)
FetchContent_Declare(GSL
    GIT_REPOSITORY "https://github.com/microsoft/GSL"
    GIT_TAG "v4.0.0"
    GIT_SHALLOW ON
)

FetchContent_MakeAvailable(GSL)

find_package(fmt        REQUIRED)
find_package(Eigen3     REQUIRED)
find_package(spdlog     REQUIRED)
find_package(CGAL       REQUIRED)

# find_program(CLANGTIDY clang-tidy)
# set(CMAKE_CXX_CLANG_TIDY
#     clang-tidy --config-file=${CMAKE_CURRENT_LIST_DIR}/.clang-tidy
# )


message(STATUS "spdlog dir: ${spdlog_DIR}")
message(STATUS "spdlog version: ${spdlog_VERSION}")
message(STATUS "fmt dir: ${fmt_DIR}")
message(STATUS "fmt version: ${fmt_VERSION}")

set(MDV_INCLUDES_DIR ${CMAKE_CURRENT_LIST_DIR}/include)

message(STATUS "GSL dir: ${GSL_DIR}")
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/utils)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/mesh)

add_executable(
    iterator_test
    ${CMAKE_CURRENT_LIST_DIR}/tests/iterator.cpp
)
target_link_libraries(
    iterator_test
    meshlib
)
