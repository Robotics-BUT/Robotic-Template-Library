cmake_minimum_required(VERSION 3.13)
project(RoboticTemplateLibrary)

enable_testing()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE  "-O3")

include_directories(include)

if(ENABLE_TESTS)
    add_subdirectory(test)
endif()

if(ENABLE_EXAMPLES)
    add_subdirectory(examples)
endif()