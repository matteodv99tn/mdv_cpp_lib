add_library(matplotlibcpp INTERFACE)
find_package(Python3 COMPONENTS Interpreter Development)
if(${Python3_FOUND})
    target_link_libraries(
        matplotlibcpp 
        INTERFACE
        Python3::Python
        Python3::Module
    )
    message(STATUS "Python3 found, matplotlib_cpp will be available")
    target_include_directories(matplotlibcpp INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/external_includes)
    target_compile_definitions(matplotlibcpp INTERFACE WITHOUT_NUMPY MATPLOTLIBAVAILABLE)

    set(MATPLOTLIB_H_DOWNLOAD_PATH ${CMAKE_CURRENT_BINARY_DIR}/external_includes/matplotlibcpp.h)
    if (NOT EXISTS "${MATPLOTLIB_H_DOWNLOAD_PATH}")
        message(STATUS "Downloading matplotlib_cpp header")
        file(DOWNLOAD "https://raw.githubusercontent.com/lava/matplotlib-cpp/master/matplotlibcpp.h" "${MATPLOTLIB_H_DOWNLOAD_PATH}")
    endif()
else()
    message(WARNING "Python3 not found, so won't be able to use matplotlib_cpp")
endif()
