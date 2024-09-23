find_package(nlohmann_json QUIET)

if (nlohmann_json_FOUND)
    message(STATUS "Found nlohmann_json installed locally")
else()
    message(STATUS "nlohmann_json not found, downloading...")
    include(FetchContent)
    FetchContent_Declare(
        nlohmann_json 
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
        DOWNLOAD_EXTRACT_TIMESTAMP NEW
    )
    FetchContent_MakeAvailable(nlohmann_json)
endif()
