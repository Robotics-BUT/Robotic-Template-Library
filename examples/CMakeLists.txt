cmake_minimum_required(VERSION 3.13)

project(RoboticTemplateLibrary-Examples)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE  "-O3")

include_directories(../include)

add_executable(e_bounding_box e_bounding_box.cpp)
add_executable(e_genetic_algorithm e_genetic_algorithm.cpp)
add_executable(e_kalman_filter e_kalman_filter.cpp)
add_executable(e_matrix e_matrix.cpp)
add_executable(e_munkres e_munkres.cpp)
add_executable(e_particle_filter e_particle_filter.cpp)
add_executable(e_quaternion e_quaternion.cpp)
add_executable(e_tf_tree e_tf_tree.cpp)
add_executable(e_transformations e_transformations.cpp)
add_executable(e_vectors e_vectors.cpp)
add_executable(e_latexexport e_latexexport.cpp)

