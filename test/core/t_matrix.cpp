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

#include <gtest/gtest.h>
#include <iostream>
#include <random>
#include <chrono>

#include <eigen3/Eigen/Dense>

#include "rtl/Core.h"
#include "rtl/io/StdLib.h"

#define STRING_STREAM(x) static_cast<std::ostringstream &&>((std::ostringstream() << x)).str()

template <typename T>
void matrixConstruction()
{
    std::cout<<"\nMatrix construction test:"<<std::endl;
    std::cout<<"\tDefault constructed 3x3:"<<std::endl;
    std::cout<<"\t\t"<<rtl::Matrix<3, 3, T>()<<std::endl;

    // Construction from Eigen types
    std::cout<<"\tConstruction from Eigen type identity matrix:"<<std::endl;
    std::cout<<"\t\t"<<rtl::Matrix<3, 3, T>(Eigen::Matrix<T, 3, 3>::Identity())<<std::endl;

    // static construction
    using M = rtl::Matrix<6, 6, T>;
    if (M::identity() * M::ones() + M::zeros() != M::ones()) {
        ASSERT_ANY_THROW(STRING_STREAM("Static matrix initialization fails."));
    }
    if (!M::nan().hasNaN()) {
        ASSERT_ANY_THROW(STRING_STREAM("\tNo NaNs in ::nan() initialized matrix."));
    }
}

void matrixConversion()
{
    std::cout<<"\nMatrix conversion test:"<<std::endl;
    std::cout<<"\t5 * rtl::Matrix<3, 3, int>::ones():"<<std::endl;
    rtl::Matrix<3, 3, int> mi = 5 * rtl::Matrix<3, 3, int>::ones();
    std::cout<<"\t\t"<<mi<<std::endl;

    std::cout<<"\tCast to float:"<<std::endl;
    rtl::Matrix<3, 3, float> mf = mi.cast<float>();
    std::cout<<"\t\t"<<mf<<std::endl;

    std::cout<<"\tFrom float to double:"<<std::endl;
    rtl::Matrix<3, 3, double> md = mf.cast<double>();
    std::cout<<"\t\t"<<md<<std::endl;

    std::cout<<"\tAnd back to int:"<<std::endl;
    mi = md.cast<int>();
    std::cout<<"\t\t"<<mi<<std::endl;

}

template <typename T>
void matrixOperators()
{
    std::cout<<"\nMatrix operators test:"<<std::endl;
    using M = rtl::Matrix<3, 3, T>;
    M m2, m3;
    rtl::Matrix<3, 5, T> m_w1, m_w2;
    rtl::VectorND<3, T> v1, v2;
    rtl::VectorND<4, T> v_err;

    // Addition subtraction
    std::cout<<"\tMatrix addition:"<<std::endl;
    std::cout<<"\t\tIdentity + Identity = "<<M::identity() + M::identity()<<std::endl;

    std::cout<<"\tIn-place addition: "<<std::endl;
    auto m1 = M::ones();
    m1 += M::ones();
    std::cout<<"\t\tOnes += Ones:"<<m1<<std::endl;

    std::cout<<"\tMatrix subtraction:"<<std::endl;
    m1 = M::identity();
    std::cout<<"\t\tIdentity - Identity = " <<m1 - M::identity()<<std::endl;

    std::cout<<"\tIn-place subtraction:"<<std::endl;
    m1 = M::ones();
    m1-=M::ones();
    std::cout<<"\t\tOnes -= Ones: "<<m1<<std::endl;

    std::cout<<"\tValues negation:"<<std::endl;
    std::cout<<"\t\t-Ones: "<<-M::ones()<<std::endl;

    // Matrix * something
    std::cout<<"\tMatrix*Scalar multiplication:"<<std::endl;
    std::cout<<"\t\tOnes * 3 = "<<M::ones() * 3<<std::endl;

    std::cout<<"\tMatrix/Scalar division:"<<std::endl;
    std::cout<<"\t\tOnes / 5 = "<<M::ones()/5<<std::endl;

    std::cout<<"\tMatrix*Vector multiplication:"<<std::endl;
    std::cout<<"\t\tM_ones * V_ones = "<<M::ones() * rtl::Vector3D<T>::ones()<<std::endl;

    std::cout<<"\tMatrix*Matrix multiplication:"<<std::endl;
    std::cout<<"\t\tOnes * Ones = "<<M::ones() * M::ones()<<std::endl;

    std::cout<<"\tMatrix*Matrix multiplication with differing sizes:"<<std::endl;
    std::cout<<"\t\tMatrix<> * Matrix<> = "<<rtl::Matrix<2, 4, T>() * rtl::Matrix<4 ,3 ,T>()<<std::endl;

    // Something * matrix
    std::cout<<"\tScalar*Matrix multiplication:"<<std::endl;
    std::cout<<"\t\t4 * Ones = "<< (T)4 * M::ones()<<std::endl;

    std::cout<<"\tVector*Matrix multiplication:"<<std::endl;
    std::cout<<"\t\tV_ones * M_ones = "<<rtl::Vector3D<T>::ones() * M::ones()<<std::endl;

    // In-place multiplication
    std::cout<<"\tIn-place scalar multiplication:"<<std::endl;
    m1 = M::ones();
    m1 *= 6;
    std::cout<<"\t\tOnes*=6: "<<m1<<std::endl;

    std::cout<<"\tIn-place scalar division:"<<std::endl;
    m1 = M::ones();
    m1 /= 2;
    std::cout<<"\t\tOnes/=2: "<<m1<<std::endl;

    std::cout<<"\tIn-place matrix multiplication:"<<std::endl;
    m1 = M::ones();
    m1 *= M::ones();
    std::cout<<"\t\tOnes*=Ones: "<<m1<<std::endl;

    // Row and column operations
    auto i = M::identity();
    auto j = M::ones();
    auto k = M::zeros();
    for (size_t x = 0; x < 3; x++)
    {
        j.setRow(x, i.getRow(x));
        k.setColumn(x, i.getColumn(x));
    }
    if (i != j)
    {
        ASSERT_ANY_THROW(STRING_STREAM("\tRow operations failed."));
    }
    if (i != k)
    {
        ASSERT_ANY_THROW(STRING_STREAM("\tColumn operations failed."));
    }
}

template <typename T>
void matrixLinAlg(T err)
{
    std::cout<<"\nMatrix linear algebra functions:"<<std::endl;
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<T> rnd_float(-1, 1);
    auto el_rnd_gen = [&generator, &rnd_float](){ return rnd_float(generator); };

    using M = rtl::Matrix<3, 3, T>;
    auto m3 = M::random(el_rnd_gen);
    
    // Other matrix operations
    std::cout<<"\tRandom matrix M:"<<std::endl;
    std::cout<<"\t\t"<<m3<<std::endl;
    std::cout<<"\tM transposed:"<<std::endl;
    std::cout<<"\t\t"<<m3.transposed()<<std::endl;
    std::cout<<"\tM inverted:"<<std::endl;
    std::cout<<"\t\t"<<m3.inverted()<<std::endl;
    std::cout<<"\tM eigenvalues:"<<std::endl;
    std::cout<<"\t\t"<<m3.eigenvalues()<<std::endl;
    std::cout<<"\tM eigenvectors:"<<std::endl;
    std::cout<<"\t\t"<<m3.eigenvectors()<<std::endl;
    std::cout<<"\tM determinant:"<<std::endl;
    std::cout<<"\t\t"<<m3.determinant()<<std::endl;
    std::cout<<"\tM trace:"<<std::endl;
    std::cout<<"\t\t"<<m3.trace()<<std::endl;

    if (M::distance(m3, m3.transposed().transposed()) > err) {
        ASSERT_ANY_THROW(STRING_STREAM("Excessive double-transpose error for: " << m3));
    }
    if (M::distance(m3, m3.inverted().inverted()) > err) {
        ASSERT_ANY_THROW(STRING_STREAM("Excessive double-inverse error for: " << m3));
    }
}

TEST(t_matrix_tests, general_test) {
    
    float err_f = 0.0001f;
    double err_d = 0.000001f;
    // Matrix tests
    matrixConstruction<float>();
    matrixConstruction<double>();

    matrixConversion();

    matrixOperators<float>();
    matrixOperators<double>();

    matrixLinAlg<float>(err_f);
    matrixLinAlg<double>(err_d);
}

int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}