cmake_minimum_required(VERSION 3.10)

project(mdvcpplib)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(FIND_LIBRARY_USE_LIB64_PATHS OFF)

option(BUILD_TESTING "" OFF)
option(MDV_VERBOSE_DMP "Includes verbose DMP logging in learning/executing")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(CTest)

set(CMAKE_CXX_STANDARD 20)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

list(PREPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake/install)

macro(find_and_log_required package)
    find_package(${package} REQUIRED)
    message(STATUS "Package ${package} found at ${${package}_DIR}")
endmacro()

macro(find_and_log package)
    find_package(${package})
    if(${package}_FOUND)
        message(STATUS "Package ${package} found at ${${package}_DIR}")
    else()
        message(STATUS "Package ${package} not found!")
    endif()
endmacro()

find_and_log_required(Eigen3)
find_and_log_required(spdlog)
find_and_log_required(CGAL)
find_and_log_required(fmt)
find_and_log_required(range-v3)
find_and_log_required(Microsoft.GSL)
find_and_log(rerun_sdk)

add_compile_definitions(MDV_CGAL_VERSION=${CGAL_VERSION_MAJOR})

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
add_library(
    mdvcpplib

    # General purpose
    ${CMAKE_CURRENT_LIST_DIR}/src/utils/logging.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/utils/logging_extras.cpp

    # Mesh
    ${CMAKE_CURRENT_LIST_DIR}/src/mesh/mesh.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/mesh/cgal_mesh.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/mesh/algorithm.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/mesh/face.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/mesh/point.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/mesh/vertex.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/mesh/tangent_vector.cpp

    # Riemann geometry
    ${CMAKE_CURRENT_LIST_DIR}/src/riemann_geometry/s3.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/riemann_geometry/mesh.cpp

    # DMP
    ${CMAKE_CURRENT_LIST_DIR}/src/dmp/dmp.cpp
)

if(rerun_sdk_FOUND)
    target_link_libraries(
        mdvcpplib
        PUBLIC 
        rerun_sdk
    )
    target_sources(
        mdvcpplib
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/rerun.cpp
    )
    target_compile_definitions(
        mdvcpplib
        PUBLIC
        MDV_WITH_RERUN_SDK
    )
endif()

target_include_directories(
    mdvcpplib
    PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    mdvcpplib
    PUBLIC 
    Eigen3::Eigen
    CGAL::CGAL
    Microsoft.GSL::GSL
    spdlog::spdlog
    PRIVATE 
    range-v3
)
if(${MDV_VERBOSE_DMP})
    target_compile_definitions(
        mdvcpplib
        PUBLIC
        MDV_VERBOSE_DMP    
    )
endif()


#  _____                     _        _     _           
# | ____|_  _____  ___ _   _| |_ __ _| |__ | | ___  ___ 
# |  _| \ \/ / _ \/ __| | | | __/ _` | '_ \| |/ _ \/ __|
# | |___ >  <  __/ (__| |_| | || (_| | |_) | |  __/\__ \
# |_____/_/\_\___|\___|\__,_|\__\__,_|_.__/|_|\___||___/
#                                                       
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/executables/mesh)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/executables/dmp)


#  _   _       _ _     _____         _       
# | | | |_ __ (_) |_  |_   _|__  ___| |_ ___ 
# | | | | '_ \| | __|   | |/ _ \/ __| __/ __|
# | |_| | | | | | |_    | |  __/\__ \ |_\__ \
#  \___/|_| |_|_|\__|   |_|\___||___/\__|___/
#                                            
if(BUILD_TESTING)
    find_and_log_required(GTest)
    file(GLOB UNIT_TEST_SRCS unit_tests/*.cpp)
    add_executable(
        mdv_gtests
        ${UNIT_TEST_SRCS}
    )
    target_link_libraries(
        mdv_gtests
        mdvcpplib
        GTest::GTest
        GTest::Main
    )
    add_test(gtest mdv_gtests)
endif()


#  ____   ___  ____ ____    ___           _        _ _ 
# |  _ \ / _ \/ ___|___ \  |_ _|_ __  ___| |_ __ _| | |
# | |_) | | | \___ \ __) |  | || '_ \/ __| __/ _` | | |
# |  _ <| |_| |___) / __/   | || | | \__ \ || (_| | | |
# |_| \_\\___/|____/_____| |___|_| |_|___/\__\__,_|_|_|
#                                                      
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
    install(
        TARGETS mdvcpplib
        EXPORT mdvcpplib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
    )
    install(
        DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
        DESTINATION include
    )

    ament_export_targets(mdvcpplib)
    ament_export_dependencies(
        Eigen3
        fmt
        spdlog
        CGAL
        range-v3
        Microsoft.GSL
    )
    if(rerun_sdk_FOUND)
        ament_export_dependencies(rerun_sdk)
    endif()

    ament_package()
else()
    message(STATUS "ament_cmake not found, skipping ROS2 installation")
endif()
