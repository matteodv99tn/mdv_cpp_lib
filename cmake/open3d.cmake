find_package(Open3D QUIET)

if (Open3D_FOUND)
    message(STATUS "Found Open3D installed locally")
else()
    message(STATUS "Open3D not found, downloading...")
    include(FetchContent)
    FetchContent_Declare(
        Open3D
        URL https://github.com/isl-org/Open3D/releases/download/v0.18.0/open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz
        DOWNLOAD_EXTRACT_TIMESTAMP NEW
    )
    FetchContent_MakeAvailable(Open3D)
    find_package(Open3D REQUIRED HINTS ${open3d_SOURCE_DIR})
endif()
