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
// Contact person: Ales Jelinek <Ales.Jelinek@ceitec.vutbr.cz>

#include <rtl/Core.h>
#include <iostream>

int main(int argc, char* argv[]) {

    /// Initialization

    auto mat2x2 = rtl::Matrix<2, 2, float>{};
    auto mat3x3 = rtl::Matrix<3, 3, double>{};

    auto eigenMat = rtl::Matrix{Eigen::Matrix<double, 2, 2>{}};

    auto zeroMatrix = rtl::Matrix<2, 2, float>::zeros();
    auto onesMatrix = rtl::Matrix<2, 2, float>::ones();
    auto identityMat = rtl::Matrix<2, 2, float>::identity();
    auto nanMatrix = rtl::Matrix<2, 2, float>::nan();

    /// Setters

    //mat2x2.setElement(row, col, val);
    mat2x2.setElement(0, 0, 1);
    mat2x2.setElement(0, 1, 2);
    mat2x2.setElement(1, 0, 3);
    mat2x2.setElement(1, 1, 4);

    //mat2x2.setRow(row, rtl::vector);
    mat2x2.setRow(0, {1, 2});
    mat2x2.setRow(1, {3, 4});

    //mat2x2.setColumn(col, rtl::Vector);
    mat2x2.setColumn(0, {1, 3});
    mat2x2.setColumn(0, {2, 4});


    /// Getters

    auto element00 = mat2x2.getElement(0,0);
    auto element11 = mat2x2(1,1);
    auto row0 = mat2x2.getRow(0);
    auto col1 = mat2x2.getColumn(1);


    /// Operators

    auto mat2x2_2 = rtl::Matrix<2, 2, float>{};
    mat2x2_2 = mat2x2;

    auto bool_result = mat2x2 == mat2x2_2;
    bool_result = mat2x2 != mat2x2_2;

    auto scalar_mul_res = mat2x2 * 42;
    auto vector_mul_res = mat2x2 * rtl::VectorND<2, float>{1.0, 2.0};
    auto matrix_mul_res = mat2x2 * mat2x2;

    auto scalar_div_res = mat2x2 / 42;

    auto mat_add_res = mat2x2 + mat2x2;
    auto mat_sub_res = mat2x2 - mat2x2;


    /// Matrix Operations

    auto transposed = mat2x2.transposed();
    auto mat2x2_copy = mat2x2;
    mat2x2_copy.transpose();
    bool_result = transposed == mat2x2_copy;   // true

    auto determinant = mat2x2.determinant();

    auto inverted_mat = mat2x2.inverted();
    mat2x2_copy = mat2x2;
    mat2x2_copy.invert();
    bool_result = inverted_mat == mat2x2_copy;   // true

    auto trace = mat2x2.trace();

    auto eigenValues = mat2x2.eigenvalues();
    auto eigenVectors = mat2x2.eigenvectors();

    return 0;
}