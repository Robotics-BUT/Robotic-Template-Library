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
#include <rtl/Transformation.h>
#include <iostream>

int main(int argc, char* argv[]) {


    /// Rotation

    auto rot1 = rtl::RotationND<3, double>{};   // uninitialized

    auto vec_from = rtl::VectorND<3, double>{0.0, 0.0, 0.0};
    auto vec_to = rtl::VectorND<3, double>{1.0, 0.0, 0.0};
    auto rot2 = rtl::RotationND<3, double>{vec_from, vec_to};

    auto rot3 = rtl::RotationND<3, double>::identity();

    /// Translation

    auto trans1 = rtl::TranslationND<3, double>{0.0, 0.0, 0.0};
    auto trans2 = rtl::TranslationND<3, double>::identity();
    auto trans3 = rtl::TranslationND<3, double>{rtl::VectorND<3, double>{0.0, 0.0, 0.0}};


    /// RigidTf

    auto tf1 = rtl::RigidTfND<3,double>{rot1, trans1};
    auto tf2 = rtl::RigidTfND<3,double>{rot1.rotQuaternion(), trans1.trVec()};
    auto roll = 0.0;
    auto pitch = 0.0;
    auto yaw = 0.0;
    auto tf3 = rtl::RigidTfND<3,double>{roll, pitch, yaw, trans1.trVec()};
    auto tf4 = rtl::RigidTfND<3,double>::identity();


    /// Getters

    auto rigid3D = rtl::RigidTfND<3,double>::identity();
    auto trans3D = rigid3D.tr();
    auto rot3D = rigid3D.rot();

    auto vector = trans3D.trVec();

    auto quat = rot3D.rotQuaternion();
    auto mat = rot3D.rotMat();
    auto axis = rot3D.rotAxis();
    auto angle = rot3D.rotAngle();

    double r,p,y;
    rot3D.rotRpy(r, p, y);

    auto sin = rot3D.rotSin();
    auto cos = rot3D.rotCos();


    /// Setters

    rigid3D.setTrVec(vector);
    rigid3D.setAngleAxis(angle, axis);

    trans3D.setTrVec({0.0, 0.0, 0.0});

    rot3D.setAngleAxis(angle, axis);


    /// Combining transformations

    auto rot = rtl::RotationND<2, double>::identity();
    auto trans = rtl::TranslationND<2, double>::identity();
    auto tf = rtl::RigidTfND<2,double>::identity();

    auto rot_rot = rot(rot);    /// apply rotation on rotation -> rotation
    auto rot_trans = rot(trans);    /// apply rotation on translation -> rigidTF
    auto rot_tf = rot(tf);    /// apply rotation on rigidTF -> rigidTF

    auto trans_rot = trans(rot);     /// applying translation on rotation -> rigidTF
    auto trans_trans = trans(trans); /// applying translation on translation -> translation
    auto trans_tf = trans(tf);      // applying translation on rigidTF -> rigidTF

    auto tf_rot = tf(rot);   /// applying rigidTF on rotation -> rigidTF
    auto tf_trans = tf(trans);   /// applying rigidTF on translation -> rigidTF
    auto tf_tf = tf(tf);   /// applying rigidTF on rigidTF -> rigidTF

    auto rot2d = rtl::RotationND<2, double>::identity();
    auto rot3d = rtl::RotationND<3, double>::identity();
    //auto rot2d_rot3d = rot2d(rot3d);      /// Combining transformations with different dimension is not possible!


    /// Applying tfs on other entities;

    auto tf3D = rtl::RigidTfND<3, double>::identity();

    // Vector
    auto vector3D = rtl::VectorND<3, double>{1.0, 0.0, 0.0};
    auto transformedVector = vector3D.transformed(tf3D);
    transformedVector = tf3D(vector3D);

    // BBx
    auto bbx3D = rtl::BoundingBoxND<3,double>{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
    auto transformedBBx = bbx3D.transformed(tf3D);
    transformedBBx = tf3D(bbx3D);


    // Frustum3D
    auto frustum3D = rtl::Frustum3D<double>{{0,0,0}, {10,1,1}, {10,-1,1}, {10,1,-1}, {10,-1,-1}, 1};
    auto transformedFrustum = frustum3D.transformed(tf3D);
    transformedFrustum = tf3D(frustum3D);


    return 0;
}