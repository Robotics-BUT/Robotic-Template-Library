cmake_minimum_required(VERSION 3.15)
project(RoboticTemplateLibrary)

enable_testing()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE  "-O3")

set(RTL_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/LaTeXTikz2D.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/LaTeXTikz3D.cpp
        )

include_directories(include)

add_library(rtl STATIC ${RTL_SOURCES})

add_subdirectory(test)
