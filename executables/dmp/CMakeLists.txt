add_executable(
    s3_test
    ${CMAKE_CURRENT_LIST_DIR}/s3_test.cpp
)
target_link_libraries(
    s3_test
    mdvlib
)

add_executable(
    scalar_test
    ${CMAKE_CURRENT_LIST_DIR}/scalar_test.cpp
)
target_link_libraries(
    scalar_test
    mdvlib
)
