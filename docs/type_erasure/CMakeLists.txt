cmake_minimum_required(VERSION 3.10)
project(type_erasure_example)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(fmt REQUIRED)

set(CMAKE_CXX_STANDARD 20)


add_executable(
   dyn_poly
   src/dynamic_polymorphism.cpp
)
target_link_libraries(
    dyn_poly
    fmt::fmt
)

add_executable(
   template_poly
   src/template_polymorphism.cpp
)
target_link_libraries(
    template_poly
    fmt::fmt
)

add_executable(
   concept_poly
   src/concept_polymorphism.cpp
)
target_link_libraries(
    concept_poly
    fmt::fmt
)

add_executable(
   value_poly
   src/value_based_polymorphism.cpp
)
target_link_libraries(
    value_poly
    fmt::fmt
)
