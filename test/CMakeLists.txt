cmake_minimum_required(VERSION 3.15)

find_package(GTest REQUIRED)
enable_testing()

add_executable(t_boundingbox t_boundingbox.cpp)
target_link_libraries(t_boundingbox GTest::GTest GTest::Main)
add_test(NAME t_boundingbox COMMAND t_boundingbox)

add_executable(t_frustum t_frustum.cpp)
target_link_libraries(t_frustum GTest::GTest GTest::Main)
add_test(NAME t_frustum COMMAND t_frustum)

add_executable(t_vectorxx t_vectorxx.cpp)
add_test(NAME t_vectorxx COMMAND t_vectorxx)

add_executable(t_matrix t_matrix.cpp)
add_test(NAME t_matrix COMMAND t_matrix)

add_executable(t_transformationxx t_transformationxx.cpp )
add_test(NAME t_transformationxx COMMAND t_transformationxx)

add_executable(t_quaternion t_quaternion.cpp )
add_test(NAME t_quaternion COMMAND t_quaternion)

add_executable(t_linesegmentxx t_linesegmentxx.cpp )
add_test(NAME t_linesegmentxx COMMAND t_linesegmentxx)

add_executable(t_vectorization t_vectorization.cpp ${RTL_SOURCES})
add_test(NAME t_vectorization COMMAND t_vectorization)

add_executable(t_latexexport t_latexexport.cpp ${RTL_SOURCES})
add_test(NAME t_latexexport COMMAND t_latexexport)

add_executable(t_io_stdlib t_io_stdlib.cpp ${RTL_SOURCES})
add_test(NAME t_io_stdlib COMMAND t_io_stdlib)

add_executable(t_segmentation t_segmentation.cpp ${RTL_SOURCES})
add_test(NAME t_segmentation COMMAND t_segmentation)
