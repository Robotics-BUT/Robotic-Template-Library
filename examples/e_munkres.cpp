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

#include <rtl/Algorithms.h>
#include <iostream>

int main() {

    auto cost_matrix = rtl::Matrix<3, 3, size_t>::zeros();
    cost_matrix.setRow(0, rtl::VectorND<3, size_t>{1, 2, 3});
    cost_matrix.setRow(1, rtl::VectorND<3, size_t>{4, 2, 6});
    cost_matrix.setRow(2, rtl::VectorND<3, size_t>{6, 3, 9});

    std::cout << " Const matrix: " << std::endl;
    for (size_t r = 0 ; r < cost_matrix.rowNr() ; r+=1) {
        for (size_t c = 0 ; c < cost_matrix.colNr() ; c+=1) {
            std::cout << cost_matrix.getElement(r, c) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    std::cout << " Minimalization task: " << std::endl;
    auto results = rtl::Munkres<size_t, 3>::solve(cost_matrix);
    float score_sum = 0.0f;
    for (const auto& result : results) {
        std::cout << "  row: " << result.row << " -> col: " << result.col << ", cost: " << result.cost << std::endl;
        score_sum += result.cost;
    }
    std::cout << "  cost sum: " << score_sum << std::endl;
    std::cout << std::endl;



    auto cost_matrix_2 = rtl::Matrix<4, 4, float>::zeros();
    cost_matrix_2.setRow(0, rtl::VectorND<4, float>{0.8f, 0.0f, 0.0f, 0.0f});
    cost_matrix_2.setRow(1, rtl::VectorND<4, float>{0.0f, 0.0f, 0.65f, 0.1f});
    cost_matrix_2.setRow(2, rtl::VectorND<4, float>{0.0f, 0.0f, 0.0f, 0.0f});
    cost_matrix_2.setRow(3, rtl::VectorND<4, float>{0.1f, 0.7f, 0.0f, 0.0f});


    std::cout << " Const matrix: " << std::endl;
    for (size_t r = 0 ; r < cost_matrix_2.rowNr() ; r+=1) {
        for (size_t c = 0 ; c < cost_matrix_2.colNr() ; c+=1) {
            std::cout << cost_matrix_2.getElement(r, c) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    std::cout << " Maximalization task: " << std::endl;
    auto results_2 = rtl::Munkres<float, 4>::solve(cost_matrix_2, true);
    score_sum = 0.0;
    for (const auto& result : results_2) {
        std::cout << "  row (new object): " << result.row << " -> col (previous object): " << result.col << ", IoU: " << result.cost << std::endl;
        score_sum += result.cost;
    }
    std::cout << "  cost sum: " << score_sum << std::endl;

    return 0;
}