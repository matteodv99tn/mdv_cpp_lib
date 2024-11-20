cmake_minimum_required(VERSION 3.10)

project(mdvlib)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

# Fetch pre-compiled open3d library
# include(cmake/open3d.cmake)
# include(cmake/range-v3.cmake) 
# include(cmake/autodiff.cmake)
# include(cmake/nlohmann_json.cmake) 
# include(cmake/matplotlibcpp.cmake) 

find_package(fmt        REQUIRED)
find_package(Eigen3     REQUIRED)
find_package(spdlog     REQUIRED)
find_package(CGAL       REQUIRED)
find_package(range-v3   REQUIRED)
find_package(rerun_sdk  REQUIRED)
find_package(Microsoft.GSL CONFIG REQUIRED)
find_package(Catch2 3   REQUIRED)

# find_program(CLANGTIDY clang-tidy)
# set(CMAKE_CXX_CLANG_TIDY
#     clang-tidy --config-file=${CMAKE_CURRENT_LIST_DIR}/.clang-tidy
# )

configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/include/mdv/config.hpp.in
    ${CMAKE_CURRENT_LIST_DIR}/include/mdv/config.hpp
)


#  __  __       _         _     _ _                          
# |  \/  | __ _(_)_ __   | |   (_) |__  _ __ __ _ _ __ _   _ 
# | |\/| |/ _` | | '_ \  | |   | | '_ \| '__/ _` | '__| | | |
# | |  | | (_| | | | | | | |___| | |_) | | | (_| | |  | |_| |
# |_|  |_|\__,_|_|_| |_| |_____|_|_.__/|_|  \__,_|_|   \__, |
#                                                      |___/ 
set(MDV_SRCS "")
set(MDV_PUB_LIBS "")
set(MDV_PRIV_LIBS "")
set(MDV_PRIV_INCLUDES_DIRS "")

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/utils)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/mesh)

list(REMOVE_DUPLICATES MDV_SRCS)
list(REMOVE_DUPLICATES MDV_PUB_LIBS)
list(REMOVE_DUPLICATES MDV_PRIV_LIBS)
list(REMOVE_DUPLICATES MDV_PRIV_INCLUDES_DIRS)

message(VERBOSE "Library sources:")
foreach(SRC ${MDV_SRCS})
    message(VERBOSE "  ${SRC}")
endforeach()
message(VERBOSE "Private includes directories:")
foreach(DIR ${MDV_PRIV_INCLUDES_DIRS})
    message(VERBOSE "  ${DIR}")
endforeach()
message(VERBOSE "Library publicly linked libraries:")
foreach(LIB ${MDV_PUB_LIBS})
    message(VERBOSE "  ${LIB}")
endforeach()
message(VERBOSE "Library privately linked libraries:")
foreach(LIB ${MDV_PRIV_LIBS})
    message(VERBOSE "  ${LIB}")
endforeach()

# Build the library target
add_library(
    mdvlib
    ${MDV_SRCS}
)
target_include_directories(
    mdvlib
    PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include
    PRIVATE ${MDV_PRIV_INCLUDES_DIRS}
)
target_link_libraries(
    mdvlib
    PUBLIC ${MDV_PUB_LIBS}
    PRIVATE ${MDV_PRIV_LIBS}
)


#  _____                     _        _     _           
# | ____|_  _____  ___ _   _| |_ __ _| |__ | | ___  ___ 
# |  _| \ \/ / _ \/ __| | | | __/ _` | '_ \| |/ _ \/ __|
# | |___ >  <  __/ (__| |_| | || (_| | |_) | |  __/\__ \
# |_____/_/\_\___|\___|\__,_|\__\__,_|_.__/|_|\___||___/
#                                                       

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/executables/mesh)


#  _   _       _ _     _____         _       
# | | | |_ __ (_) |_  |_   _|__  ___| |_ ___ 
# | | | | '_ \| | __|   | |/ _ \/ __| __/ __|
# | |_| | | | | | |_    | |  __/\__ \ |_\__ \
#  \___/|_| |_|_|\__|   |_|\___||___/\__|___/
#                                            
file(GLOB UNIT_TEST_SRCS unit_tests/*.cpp)
add_executable(
    catch2_tests
    ${UNIT_TEST_SRCS}
)
target_include_directories(
    catch2_tests
    PUBLIC
    ${MDV_PRIV_INCLUDES_DIRS}
)
target_link_libraries(
    catch2_tests
    PRIVATE
    Catch2::Catch2WithMain
    mdvlib
)
