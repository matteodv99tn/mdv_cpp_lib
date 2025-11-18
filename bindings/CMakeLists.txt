set(CMAKE_CXX_STANDARD 20)
find_package(SWIG REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
include(UseSWIG)

# set(CMAKE_SWIG_FLAGS -WAll)

function(swig_lib_module_wrapper MODULE_NAME)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs SOURCES)
    cmake_parse_arguments(MOD "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT MOD_SOURCES)
        message(FATAL_ERROR "swig_lib_module_wrapper(${MODULE_NAME}) requires SOURCES to be specified.")
    endif()

    foreach(SRC ${MOD_SOURCES})
        set_property(SOURCE ${SRC} PROPERTY CPLUSPLUS ON)
    endforeach()

    message(STATUS "Adding SWIG module ${MODULE_NAME} -- Sources: ${MOD_SOURCES}")
    swig_add_library(
        ${MODULE_NAME}
        TYPE SHARED
        LANGUAGE python
        OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/python
        SOURCES ${MOD_SOURCES}
    )
    
    target_link_libraries(
        ${MODULE_NAME}
        mdvcpplib
        Python3::Python
        Python3::NumPy
    )
    
    set_property(TARGET ${MODULE_NAME} PROPERTY SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON)
    set_target_properties(${MODULE_NAME} PROPERTIES
            INSTALL_RPATH "$ORIGIN"
            INSTALL_RPATH_USE_ORIGIN TRUE
    )
    install(
        TARGETS ${MODULE_NAME}
        LIBRARY DESTINATION mdv
    )
endfunction()

install(
    TARGETS mdvcpplib
    LIBRARY DESTINATION mdv
)

install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python/
    DESTINATION mdv
)
