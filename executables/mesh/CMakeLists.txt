include(FetchContent)
FetchContent_Declare(
    matioCpp
    URL https://github.com/ami-iit/matio-cpp/archive/refs/tags/v0.3.0.zip
)
FetchContent_MakeAvailable(matioCpp)

add_executable(
    mesh_visualiser
    ${CMAKE_CURRENT_LIST_DIR}/mesh_visualiser.cpp
)
target_link_libraries(
    mesh_visualiser
    mdvcpplib
)

add_executable(
    pd
    ${CMAKE_CURRENT_LIST_DIR}/pd.cpp
)
target_link_libraries(
    pd
    mdvcpplib
)

add_executable(
    mesh-gp
    ${CMAKE_CURRENT_LIST_DIR}/mesh-gp.cpp
)
target_link_libraries(
    mesh-gp
    mdvcpplib
    matioCpp::matioCpp
    range-v3
)
