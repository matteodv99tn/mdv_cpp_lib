find_package(autodiff QUIET)

if(autodiff_FOUND)
    message(STATUS "Found autodiff installed locally")
else()
    message(STATUS "autodiff not found, downloading...")
    FetchContent_Declare(
        autodiff
        URL https://github.com/autodiff/autodiff/archive/refs/tags/v1.1.2.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP NEW
    )
    set(AUTODIFF_BUILD_TESTS OFF)
    set(AUTODIFF_BUILD_PYTHON OFF)
    set(AUTODIFF_BUILD_EXAMPLES OFF)
    set(AUTODIFF_BUILD_DOCS OFF)

    FetchContent_MakeAvailable(autodiff)
endif()
