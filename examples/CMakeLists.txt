cmake_minimum_required(VERSION 3.13)

project(Tests)

if(ENABLE_USAGE_EXAMPLES)

    include_directories(../include)

    add_executable(e_bounding_box e_bounding_box.cpp)
    add_executable(e_matrix e_matrix.cpp)
    add_executable(e_quaternion e_quaternion.cpp)
    add_executable(e_tf_tree e_tf_tree.cpp)
    add_executable(e_transformations e_transformations.cpp)
    add_executable(e_vectors e_vectors.cpp)
    add_executable(e_latexexport e_latexexport.cpp)

ENDIF()
