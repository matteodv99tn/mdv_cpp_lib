find_package(range-v3 QUIET)

if(range-v3_FOUND)
    message(STATUS "Found range-v3 installed locally")
else()
    message(STATUS "range-v3 not found, downloading...")
    FetchContent_Declare(
        range-v3
        URL https://github.com/ericniebler/range-v3/archive/refs/tags/0.12.0.zip
        DOWNLOAD_EXTRACT_TIMESTAMP NEW
    )
    FetchContent_MakeAvailable(range-v3)
endif()
