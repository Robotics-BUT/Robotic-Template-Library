// This file is part of the Robotic Template Library (RTL), a C++
// template library for usage in robotic research and applications
// under the MIT licence:
//
// Copyright 2020 Brno University of Technology
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom
// the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
// OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
// OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Contact person: Adam Ligocki <adam.ligocki@vutbr.cz>

#include <gtest/gtest.h>
#include "rtl/Core.h"
#include <utility>
#include <iostream>

template<typename T, std::size_t N>
constexpr T array_sum( std::array<T,N> array) {
    T sum = 0;
    for (std::size_t i = 0; i < N; i++) {
        sum += array[i];
    }
    return sum;
}

template<typename T, std::size_t N>
constexpr T array_product( std::array<T,N> array) {
    T sum = 0;
    for (std::size_t i = 0; i < N; i++) {
        sum += array[i];
    }
    return sum;
}


TEST(t_occupancymap_tests, initial) {
    rtl::Occupancy2Df map{{10, 10}, {1.0f, 1.0f}};
    for (size_t i = 0 ; i < 10 ; i++) {
        for (size_t j = 0 ; j < 10 ; j++) {
            map.setCell(i*j, {i,j});
        }
    }

    for (size_t i = 0 ; i < 10 ; i++) {
        for (size_t j = 0 ; j < 10 ; j++) {
            std::cout << map.getCell({i,j}) << " ";
        }
        std::cout << std::endl;
    }
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}