
add_executable(
    mesh_visualiser
    ${CMAKE_CURRENT_LIST_DIR}/mesh_visualiser.cpp
)
target_link_libraries(
    mesh_visualiser
    mdvlib
)
