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
    range-v3
)

add_executable(
    mesh_planar_morphism
    ${CMAKE_CURRENT_LIST_DIR}/mesh_planar_morphism.cpp
)
target_link_libraries(
    mesh_planar_morphism
    mdvcpplib
)
