// This file is part of the Robotic Template Library (RTL), a C++
// template library for usage in robotic research and applications
// under the MIT licence:
//
// Copyright 2021 Brno University of Technology
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

#include "rtl/Algorithms.h"

#define error_1 1e-1

TEST(t_genetic_algorithm, init) {
    auto genetic_algorithm = rtl::GeneticAlgorithm<rtl::SimpleAgent<float>, 100, 10, 50, 10>();
}


TEST(t_genetic_algorithm, test_float_1) {
    auto genetic_algorithm = rtl::GeneticAlgorithm<rtl::SimpleAgent<float>, 100, 10, 50, 50>();

    rtl::SimpleAgent<float>::set_fit_fn([](float val){
        return 1.0f / (std::abs(0.0 - val) + 0.001f);
    });

    for (size_t i = 0 ; i < 10 ; i++) {
        genetic_algorithm.iterate_epoch();
    }

    auto best = genetic_algorithm.best_agent();
    EXPECT_NEAR(0.0f, best.value(), error_1);
}


TEST(t_genetic_algorithm, test_float_2) {
    auto genetic_algorithm = rtl::GeneticAlgorithm<rtl::SimpleAgent<float>, 1000, 100, 500, 500>();

    rtl::SimpleAgent<float>::set_fit_fn([](float val){
        return 1.0f / (std::abs(0.0 - val) + 0.001f);
    });

    for (size_t i = 0 ; i < 100 ; i++) {
        genetic_algorithm.iterate_epoch();
    }

    auto best = genetic_algorithm.best_agent();
    EXPECT_NEAR(0.0f, best.value(), error_1);
}


TEST(t_genetic_algorithm, test_float_3) {
    auto genetic_algorithm = rtl::GeneticAlgorithm<rtl::SimpleAgent<float>, 10000, 1000, 5000, 5000>();

    rtl::SimpleAgent<float>::set_fit_fn([](float val){
        return 1.0f / (std::abs(0.0 - val) + 0.001f);
    });

    for (size_t i = 0 ; i < 1000 ; i++) {
        genetic_algorithm.iterate_epoch();
    }

    auto best = genetic_algorithm.best_agent();
    EXPECT_NEAR(0.0f, best.value(), error_1);
}

TEST(t_genetic_algorithm, test_int_1) {
    auto genetic_algorithm = rtl::GeneticAlgorithm<rtl::SimpleAgent<int>, 100, 10, 50, 50>();

    rtl::SimpleAgent<int>::set_fit_fn([](int val){
        return 1.0f / (std::abs(0.0 - val) + 0.001f);
    });

    for (size_t i = 0 ; i < 10 ; i++) {
        genetic_algorithm.iterate_epoch();
    }

    auto best = genetic_algorithm.best_agent();
    EXPECT_NEAR(0, best.value(), error_1);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
