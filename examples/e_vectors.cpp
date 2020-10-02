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

    /// Vector initialization
    /// Use predefined vector types in the most common cases (rtl::Vector2D<data_type> and rtl::Vector3D<data_type>)
    /// and custom defined for special purposes (rtl::VectorND<dim, data_type>)

    auto vector2Dfloat = rtl::Vector2D<float>(1.0, 1.0);
    auto vector3Ddouble = rtl::Vector3D<float>(1.0, 1.0, 1.0);
    auto vector5Dint = rtl::VectorND<5, int>(1, 2, 3, 4, 5);
    auto ones = rtl::VectorND<100, int>::ones();
    auto zeros = rtl::Vector3d::zeros();

    /// Access to vector raw data

    auto data2D = vector2Dfloat.data();
    auto data5D = vector5Dint.data();


    /// Getters and setters

    vector3Ddouble.setElement(0, 123.0);
    auto element0 = vector3Ddouble.getElement(0);

    vector3Ddouble.setX(123.0);
    element0 = vector3Ddouble.x();

    vector5Dint.setElement(0, 5);
    auto intElement = vector5Dint.getElement(0);


    /// [] operator

    vector5Dint[4] = 20;
    vector2Dfloat[1] = 0.0;


    /// Lenght and Normalization

    auto veryLongVector = rtl::Vector3f{1000.0, 1000.0, 1000.0}; // lenght == 1732.05
    std::cout << veryLongVector.length() << std::endl;
    std::cout << veryLongVector.lengthSquared() << std::endl;

    auto normalizedVector = veryLongVector.normalized();
    veryLongVector.normalize();
    std::cout << normalizedVector.x() << " " << normalizedVector.y() << " " << normalizedVector.z() <<std::endl;


    /// Arithmetic operators

    auto a = rtl::Vector2f {2.0, 5.0};
    auto b = rtl::Vector2f {-1.0, 3.0};

    auto add = a + b;
    auto sub = a - b;
    auto mul = a * 2.0;
    auto div = a / 2.0;

    a += b;
    a -= b;
    a *= 2.0;
    a /= 2.0;

    auto dotProduct = a.dot(b);


    /// Comparison operators

    auto x = rtl::Vector3d {1.0, 1.0, 1.0};
    auto y = rtl::Vector3d {1.0, 1.0, 1.0};
    auto z = rtl::Vector3d {2.0, 2.0, 2.0};

    std::cout << (x == y ? "X == Y" : "X != Y") << std::endl;
    std::cout << (x != z ? "X != Z" : "X == Z") << std::endl;


    /// Other static methods

    auto k = rtl::Vector2f {1.0, 0.0};
    auto j = rtl::Vector2f {0.0, 1.0};

    auto angle = rtl::Vector2f::angleShortest(j ,k); // pi/2
    auto distance = rtl::Vector2f::distance(j,k);   // 1.42

    /// ...

    std::cout << angle << " " << distance << std::endl;

    return 0;
}