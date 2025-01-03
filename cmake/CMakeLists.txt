cmake_minimum_required(VERSION 3.20)
project(mdvcpplib_dependencies)

include(ExternalProject)
include(FetchContent)

set(PROJECT_BUILD_DIR ${CMAKE_BINARY_DIR})
set(PROJECT_INSTALL_DIR ${CMAKE_SOURCE_DIR}/install)

find_package(range-v3)
if(NOT range-v3_FOUND)
    ExternalProject_Add(
        range-v3-dep
        DOWNLOAD_EXTRACT_TIMESTAMP NEW
        URL https://github.com/ericniebler/range-v3/archive/refs/tags/0.12.0.tar.gz
        PREFIX ${PROJECT_BUILD_DIR}/range-v3
        INSTALL_DIR ${PROJECT_INSTALL_DIR}/range-v3
        CMAKE_ARGS 
            -DCMAKE_INSTALL_PREFIX=${PROJECT_INSTALL_DIR}/range-v3
            -DCMAKE_BUILD_TYPE=Release
            -DRANGE_V3_TESTS=OFF
            -DRANGE_V3_EXAMPLES=OFF
            -DRANGE_V3_DOCS=OFF
    )
endif()

find_package(Microsoft.GSL)
if(NOT Microsoft.GSL_FOUND)
    ExternalProject_Add(
        gsl-dep
        DOWNLOAD_EXTRACT_TIMESTAMP NEW
        URL https://github.com/microsoft/GSL/archive/refs/tags/v4.1.0.tar.gz
        PREFIX ${PROJECT_BUILD_DIR}/gsl
        INSTALL_DIR ${PROJECT_INSTALL_DIR}/gsl
        CMAKE_ARGS 
            -DCMAKE_INSTALL_PREFIX=${PROJECT_INSTALL_DIR}/Microsoft.GSL
            -DCMAKE_BUILD_TYPE=Release
            -DGSL_INSTALL=ON
            -DGSL_TEST=OFF
    )
endif()

find_package(Catch2)
if(NOT Catch2_FOUND)
    ExternalProject_Add(
        Catch2-dep
        DOWNLOAD_EXTRACT_TIMESTAMP NEW
        URL https://github.com/catchorg/Catch2/archive/refs/tags/v3.7.0.tar.gz
        PREFIX ${PROJECT_BUILD_DIR}/Catch2
        INSTALL_DIR ${PROJECT_INSTALL_DIR}/Catch2
        CMAKE_ARGS 
        -DCMAKE_INSTALL_PREFIX=${PROJECT_INSTALL_DIR}/Catch2
            -DCMAKE_BUILD_TYPE=Release
            -DJSON_BuildTests=OFF
            -DJSON_Install=ON
    )
endif()

find_package(rerun_sdk)
if(NOT rerun_sdk_FOUND)
    ExternalProject_Add(
        rerun_sdk-dep
        DOWNLOAD_EXTRACT_TIMESTAMP NEW
        URL https://github.com/rerun-io/rerun/releases/download/0.20.3/rerun_cpp_sdk.zip
        PREFIX ${PROJECT_BUILD_DIR}/rerun_sdk
        INSTALL_DIR ${PROJECT_INSTALL_DIR}/rerun_sdk
        CMAKE_ARGS 
            -DCMAKE_INSTALL_PREFIX=${PROJECT_INSTALL_DIR}/rerun_sdk
            -DCMAKE_BUILD_TYPE=Release
    )
endif()

find_package(fmt)
if(NOT fmt_FOUND)
    ExternalProject_Add(
        fmt-dep
        DOWNLOAD_EXTRACT_TIMESTAMP NEW
        URL https://github.com/fmtlib/fmt/archive/refs/tags/11.1.1.tar.gz
        PREFIX ${PROJECT_BUILD_DIR}/fmt
        INSTALL_DIR ${PROJECT_INSTALL_DIR}/fmt
        CMAKE_ARGS 
            -DCMAKE_INSTALL_PREFIX=${PROJECT_INSTALL_DIR}/fmt
            -DCMAKE_BUILD_TYPE=Release
            -DFMT_DOC=OFF
            -DFMT_TEST=OFF
    )
endif()

find_package(spdlog)
if(NOT spdlog_FOUND)
    ExternalProject_Add(
        spdlog-dep
        DOWNLOAD_EXTRACT_TIMESTAMP NEW
        URL https://github.com/gabime/spdlog/archive/refs/tags/v1.14.1.tar.gz
        PREFIX ${PROJECT_BUILD_DIR}/spdlog
        INSTALL_DIR ${PROJECT_INSTALL_DIR}/spdlog
        CMAKE_ARGS 
            -DCMAKE_INSTALL_PREFIX=${PROJECT_INSTALL_DIR}/spdlog
            -DCMAKE_BUILD_TYPE=Release
    )
endif()
