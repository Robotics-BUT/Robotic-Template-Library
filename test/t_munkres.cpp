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
#include <algorithm>

#include "rtl/Algorithms.h"

#define max_err 1e-10

TEST(t_munkres, test_1) {

    auto cost_matrix = rtl::Matrix<3, 3, size_t>::zeros();
    cost_matrix.setRow(0, rtl::VectorND<3, size_t>{1, 2, 3});
    cost_matrix.setRow(1, rtl::VectorND<3, size_t>{2, 4, 6});
    cost_matrix.setRow(2, rtl::VectorND<3, size_t>{3, 6, 9});

    auto result = rtl::Munkres<size_t, 3>::solve(cost_matrix);

    EXPECT_EQ(result[0].worker, 0); EXPECT_EQ(result[0].job, 2); EXPECT_EQ(result[0].cost, 3);
    EXPECT_EQ(result[1].worker, 1); EXPECT_EQ(result[1].job, 1); EXPECT_EQ(result[1].cost, 4);
    EXPECT_EQ(result[2].worker, 2); EXPECT_EQ(result[2].job, 0); EXPECT_EQ(result[2].cost, 3);
}

TEST(t_munkres, test_1_max) {

    auto cost_matrix = rtl::Matrix<3, 3, size_t>::zeros();
    cost_matrix.setRow(0, rtl::VectorND<3, size_t>{1, 2, 3});
    cost_matrix.setRow(1, rtl::VectorND<3, size_t>{2, 4, 6});
    cost_matrix.setRow(2, rtl::VectorND<3, size_t>{3, 6, 9});

    auto result = rtl::Munkres<size_t, 3>::solve(cost_matrix, true);

    EXPECT_EQ(result[0].worker, 0); EXPECT_EQ(result[0].job, 0); EXPECT_EQ(result[0].cost, 1);
    EXPECT_EQ(result[1].worker, 1); EXPECT_EQ(result[1].job, 1); EXPECT_EQ(result[1].cost, 4);
    EXPECT_EQ(result[2].worker, 2); EXPECT_EQ(result[2].job, 2); EXPECT_EQ(result[2].cost, 9);
}

TEST(t_munkres, test_2) {

    auto cost_matrix = rtl::Matrix<6, 6, int>::zeros();
    cost_matrix.setRow(0, rtl::VectorND<6, int>{22, 14, 120, 21, 4, 51});
    cost_matrix.setRow(1, rtl::VectorND<6, int>{19, 12, 172, 21, 28, 43});
    cost_matrix.setRow(2, rtl::VectorND<6, int>{161, 122, 2, 50, 128, 39});
    cost_matrix.setRow(3, rtl::VectorND<6, int>{19, 22, 90, 11, 28, 4});
    cost_matrix.setRow(4, rtl::VectorND<6, int>{1, 30, 113, 14, 28, 86});
    cost_matrix.setRow(5, rtl::VectorND<6, int>{60, 70, 170, 28, 68, 104});

    auto result = rtl::Munkres<int, 6>::solve(cost_matrix);

    EXPECT_EQ(result[0].worker, 0); EXPECT_EQ(result[0].job, 4); EXPECT_EQ(result[0].cost, 4);
    EXPECT_EQ(result[1].worker, 1); EXPECT_EQ(result[1].job, 1); EXPECT_EQ(result[1].cost, 12);
    EXPECT_EQ(result[2].worker, 2); EXPECT_EQ(result[2].job, 2); EXPECT_EQ(result[2].cost, 2);
    EXPECT_EQ(result[3].worker, 3); EXPECT_EQ(result[3].job, 5); EXPECT_EQ(result[3].cost, 4);
    EXPECT_EQ(result[4].worker, 4); EXPECT_EQ(result[4].job, 0); EXPECT_EQ(result[4].cost, 1);
    EXPECT_EQ(result[5].worker, 5); EXPECT_EQ(result[5].job, 3); EXPECT_EQ(result[5].cost, 28);
}


TEST(t_munkres, test_3_max) {

    auto cost_matrix = rtl::Matrix<4, 4, float>::zeros();
    cost_matrix.setRow(0, rtl::VectorND<4, float>{0.8f, 0.0f, 0.0f, 0.0f});
    cost_matrix.setRow(1, rtl::VectorND<4, float>{0.0f, 0.0f, 0.65f, 0.1f});
    cost_matrix.setRow(2, rtl::VectorND<4, float>{0.0f, 0.0f, 0.0f, 0.0f});
    cost_matrix.setRow(3, rtl::VectorND<4, float>{0.1f, 0.7f, 0.0f, 0.0f});

    auto result = rtl::Munkres<float, 4>::solve(cost_matrix, true);

    EXPECT_EQ(result[0].worker, 0); EXPECT_EQ(result[0].job, 0); EXPECT_EQ(result[0].cost, 0.8f);
    EXPECT_EQ(result[1].worker, 1); EXPECT_EQ(result[1].job, 2); EXPECT_EQ(result[1].cost, 0.65f);
    EXPECT_EQ(result[2].worker, 2); EXPECT_EQ(result[2].job, 3); EXPECT_EQ(result[2].cost, 0.0f);
    EXPECT_EQ(result[3].worker, 3); EXPECT_EQ(result[3].job, 1); EXPECT_EQ(result[3].cost, 0.7f);
}



int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
