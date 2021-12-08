cmake_minimum_required(VERSION 3.13)

project(RoboticTemplateLibrary-Tests)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE  "-O3")

include_directories(../include)

find_package(GTest REQUIRED)
enable_testing()

macro(make_test name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} GTest::GTest GTest::Main)
    add_test(NAME ${name} COMMAND ${name})
endmacro()

macro(make_core_test name)
    add_executable(${name} core/${name}.cpp)
    target_link_libraries(${name} GTest::GTest GTest::Main)
    add_test(NAME ${name} COMMAND ${name})
endmacro()

macro(make_alg_test name)
    add_executable(${name} alg/${name}.cpp)
    target_link_libraries(${name} GTest::GTest GTest::Main)
    add_test(NAME ${name} COMMAND ${name})
endmacro()

macro(make_tf_test name)
    add_executable(${name} tf/${name}.cpp)
    target_link_libraries(${name} GTest::GTest GTest::Main)
    add_test(NAME ${name} COMMAND ${name})
endmacro()


make_test(t_io_stdlib)
make_test(t_latexexport)
make_test(t_linesegmentxx)
make_test(t_segmentation)
make_test(t_type_traits)
make_test(t_vectorization)

make_core_test(t_boundingbox)
make_core_test(t_frustum)
make_core_test(t_matrix)
make_core_test(t_quaternion)
make_core_test(t_vectorxx)

make_alg_test(t_genetic_algorithm)
make_alg_test(t_kalman)
make_alg_test(t_munkres)
make_alg_test(t_particle_filter)

make_tf_test(t_tf_chain)
make_tf_test(t_tf_general_tf)
make_tf_test(t_tf_tree)
make_tf_test(t_tf_tree_node)
make_tf_test(t_tf_management)
make_tf_test(t_transformationxx)
