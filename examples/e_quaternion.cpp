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

    auto q1 = rtl::Quaternion<double>{}; // Uninitialized quaternion with values of dtype double
    q1 = rtl::Quaterniond{};             // the same initialization

    auto q2 = rtl::Quaternion<float>{1.0, 0.0, 0.0, 0.0}; // Float value quaternion (w, x, y, z)
    q2 = rtl::Quaternionf::identity();

    auto vector_from = rtl::VectorND<3, double>{0.0, 0.0, 0.0};
    auto vector_to = rtl::VectorND<3, double>{1.0, 0.0, 0.0};
    auto q3 = rtl::Quaternion<double>{vector_from, vector_to};

    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    auto q4 = rtl::Quaternion<double> {roll, pitch, yaw};

    /// Getters

    auto quat = rtl::Quaternion<double>::identity();
    auto eigen_quat = quat.data();  // returns eigen quaternion

    auto w = quat.w();
    auto x = quat.x();
    auto y = quat.y();
    auto z = quat.z();

    auto scalar = quat.scalar();    // returns w component
    auto vector = quat.vector();    // returns (x, y, z) vector

    quat.rpy(roll, pitch, yaw);     // returns rpy via argument


    /// Setters

    quat.setX(x);
    quat.setY(y);
    quat.setZ(z);
    quat.setW(w);

    quat.setScalar(scalar);
    quat.setVector(rtl::VectorND<3, double>{x, y, z});


    /// Operators

    auto quat1 = rtl::Quaternion<double>{1.0, 0.0, 0.0, 0.0};
    auto quat2 = rtl::Quaternion<double>{0.0, 1.0, 0.0, 0.0};

    auto slerp = quat1.slerp(quat2, 0.1);       // slerp

    auto sumQuat = quat1 + quat2;
    sumQuat += quat2;

    auto subQuat = quat1 - quat2;
    subQuat -= quat2;
    subQuat = -quat2;

    auto mulQuat = quat1 * quat2;
    mulQuat *= quat2;
    mulQuat = quat1 * 42;

    auto divQuat = quat1 / 42;

    /// Normalization

    double norm = quat1.norm();
    double normSquared = quat1.normSquared();


    auto nonNormalizedQuat = rtl::Quaternion<double>{99.0, 0.0, 0.0, 0.0};
    auto normalizedQuat = nonNormalizedQuat.normalized();

    nonNormalizedQuat.normalize();
    normalizedQuat = nonNormalizedQuat;


    /// Rotations

    quat = rtl::Quaternion<double>::identity();
    auto invertedQuat = quat1.inverted();

    quat.invert();
    invertedQuat = quat;

    auto rotationMatrix = quat.rotMat();


    return 0;
}