cmake_minimum_required(VERSION 3.13)

project(RoboticTemplateLibrary-Tests)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE  "-O3")

include_directories(../include)

find_package(GTest REQUIRED)
enable_testing()

add_executable(t_boundingbox t_boundingbox.cpp)
target_link_libraries(t_boundingbox GTest::GTest GTest::Main)
add_test(NAME t_boundingbox COMMAND t_boundingbox)

add_executable(t_frustum t_frustum.cpp)
target_link_libraries(t_frustum GTest::GTest GTest::Main)
add_test(NAME t_frustum COMMAND t_frustum)

add_executable(t_tf_chain tf/t_tf_chain.cpp)
target_link_libraries(t_tf_chain GTest::GTest GTest::Main)
add_test(NAME t_tf_chain COMMAND t_tf_chain)

add_executable(t_tf_general_tf tf/t_tf_general_tf.cpp)
target_link_libraries(t_tf_general_tf GTest::GTest GTest::Main)
add_test(NAME t_tf_general_tf COMMAND t_tf_general_tf)

add_executable(t_tf_tree tf/t_tf_tree.cpp)
target_link_libraries(t_tf_tree GTest::GTest GTest::Main)
add_test(NAME t_tf_tree COMMAND t_tf_tree)

add_executable(t_tf_tree_node tf/t_tf_tree_node.cpp)
target_link_libraries(t_tf_tree_node GTest::GTest GTest::Main)
add_test(NAME t_tf_tree_node COMMAND t_tf_tree_node)

add_executable(t_vectorxx t_vectorxx.cpp)
target_link_libraries(t_vectorxx GTest::GTest GTest::Main)
add_test(NAME t_vectorxx COMMAND t_vectorxx)

add_executable(t_matrix t_matrix.cpp)
target_link_libraries(t_matrix GTest::GTest GTest::Main)
add_test(NAME t_matrix COMMAND t_matrix)

add_executable(t_munkres t_munkres.cpp)
target_link_libraries(t_munkres GTest::GTest GTest::Main)
add_test(NAME t_munkres COMMAND t_munkres)

add_executable(t_transformationxx t_transformationxx.cpp )
target_link_libraries(t_transformationxx GTest::GTest GTest::Main)
add_test(NAME t_transformationxx COMMAND t_transformationxx)

add_executable(t_quaternion t_quaternion.cpp )
target_link_libraries(t_quaternion GTest::GTest GTest::Main)
add_test(NAME t_quaternion COMMAND t_quaternion)

add_executable(t_linesegmentxx t_linesegmentxx.cpp )
target_link_libraries(t_linesegmentxx GTest::GTest GTest::Main)
add_test(NAME t_linesegmentxx COMMAND t_linesegmentxx)

add_executable(t_vectorization t_vectorization.cpp ${RTL_SOURCES})
add_test(NAME t_vectorization COMMAND t_vectorization)

add_executable(t_latexexport t_latexexport.cpp ${RTL_SOURCES})
add_test(NAME t_latexexport COMMAND t_latexexport)

add_executable(t_io_stdlib t_io_stdlib.cpp ${RTL_SOURCES})
add_test(NAME t_io_stdlib COMMAND t_io_stdlib)

add_executable(t_segmentation t_segmentation.cpp ${RTL_SOURCES})
add_test(NAME t_segmentation COMMAND t_segmentation)

add_executable(t_tf_management tf/t_tf_management.cpp ${RTL_SOURCES})
add_test(NAME t_tf_management COMMAND t_tf_management)

add_executable(t_type_traits t_type_traits.cpp ${RTL_SOURCES})
add_test(NAME t_type_traits COMMAND t_type_traits)

