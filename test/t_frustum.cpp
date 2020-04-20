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

#include "rtl/Core.h"
#include "rtl/Transformation.h"

#define max_err 1e-10

TEST(t_frustum, init) {

    using V = rtl::Frustum3D<double>::VectorType;

    rtl::Frustum3D<double> frustum(V{0,0,0}, V{10,1,1}, V{10,-1,1}, V{10,1,-1}, V{10,-1,-1}, 1);

    EXPECT_EQ(frustum.getOrigin().x(), 0);
    EXPECT_EQ(frustum.getOrigin().y(), 0);
    EXPECT_EQ(frustum.getOrigin().z(), 0);
}

TEST(t_frustum, near_getters) {

    using V = rtl::Frustum3D<double>::VectorType;

    rtl::Frustum3D<double> frustum(V{0,0,0}, V{10,1,1}, V{10,-1,1}, V{10,1,-1}, V{10,-1,-1}, 1);

    EXPECT_EQ(frustum.getOrigin().x(), 0);
    EXPECT_EQ(frustum.getOrigin().y(), 0);
    EXPECT_EQ(frustum.getOrigin().z(), 0);

    EXPECT_EQ(frustum.getNearTopLeft().x(), 10);
    EXPECT_EQ(frustum.getNearTopLeft().y(), 1);
    EXPECT_EQ(frustum.getNearTopLeft().z(), 1);

    EXPECT_EQ(frustum.getNearTopRight().x(), 10);
    EXPECT_EQ(frustum.getNearTopRight().y(), -1);
    EXPECT_EQ(frustum.getNearTopRight().z(), 1);

    EXPECT_EQ(frustum.getNearBottomLeft().x(), 10);
    EXPECT_EQ(frustum.getNearBottomLeft().y(), 1);
    EXPECT_EQ(frustum.getNearBottomLeft().z(), -1);

    EXPECT_EQ(frustum.getNearBottomRight().x(), 10);
    EXPECT_EQ(frustum.getNearBottomRight().y(), -1);
    EXPECT_EQ(frustum.getNearBottomRight().z(), -1);
}

TEST(t_frustum, far_getters) {

    using V = rtl::Frustum3D<double>::VectorType;

    double xDist = 10;
    double depth = 1;
    rtl::Frustum3D<double> frustum(V{0,0,0}, V{xDist,1,1}, V{xDist,-1,1}, V{xDist,1,-1}, V{xDist,-1,-1}, depth);
    double scale = ( (xDist + depth)/ xDist );

    EXPECT_EQ(frustum.getOrigin().x(), 0 * scale);
    EXPECT_EQ(frustum.getOrigin().y(), 0 * scale);
    EXPECT_EQ(frustum.getOrigin().z(), 0 * scale);

    EXPECT_EQ(frustum.getFarTopLeft().x(), 10 * scale);
    EXPECT_EQ(frustum.getFarTopLeft().y(), 1 * scale);
    EXPECT_EQ(frustum.getFarTopLeft().z(), 1 * scale);

    EXPECT_EQ(frustum.getFarTopRight().x(), 10 * scale);
    EXPECT_EQ(frustum.getFarTopRight().y(), -1 * scale);
    EXPECT_EQ(frustum.getFarTopRight().z(), 1 * scale);

    EXPECT_EQ(frustum.getFarBottomLeft().x(), 10 * scale);
    EXPECT_EQ(frustum.getFarBottomLeft().y(), 1 * scale);
    EXPECT_EQ(frustum.getFarBottomLeft().z(), -1 * scale);

    EXPECT_EQ(frustum.getFarBottomRight().x(), 10 * scale);
    EXPECT_EQ(frustum.getFarBottomRight().y(), -1 * scale);
    EXPECT_EQ(frustum.getFarBottomRight().z(), -1 * scale);
}

TEST(t_frustum, transformation_zero) {
    using V = rtl::Frustum3D<double>::VectorType;

    rtl::Transformation3D<double> tf{rtl::Quaternion<double>::identity(), rtl::Vector3D<double>{0,0,0}};
    double xDist = 10;
    double depth = 1;
    rtl::Frustum3D<double> frustum(V{0,0,0}, V{xDist,1,1}, V{xDist,-1,1}, V{xDist,1,-1}, V{xDist,-1,-1}, depth);
    double scale = ( (xDist + depth)/ xDist );
    frustum.transform(tf);

    EXPECT_EQ(frustum.getOrigin().x(), 0);
    EXPECT_EQ(frustum.getOrigin().y(), 0);
    EXPECT_EQ(frustum.getOrigin().z(), 0);

    EXPECT_EQ(frustum.getNearTopLeft().x(), 10);
    EXPECT_EQ(frustum.getNearTopLeft().y(), 1);
    EXPECT_EQ(frustum.getNearTopLeft().z(), 1);

    EXPECT_EQ(frustum.getNearTopRight().x(), 10);
    EXPECT_EQ(frustum.getNearTopRight().y(), -1);
    EXPECT_EQ(frustum.getNearTopRight().z(), 1);

    EXPECT_EQ(frustum.getNearBottomLeft().x(), 10);
    EXPECT_EQ(frustum.getNearBottomLeft().y(), 1);
    EXPECT_EQ(frustum.getNearBottomLeft().z(), -1);

    EXPECT_EQ(frustum.getNearBottomRight().x(), 10);
    EXPECT_EQ(frustum.getNearBottomRight().y(), -1);
    EXPECT_EQ(frustum.getNearBottomRight().z(), -1);

    EXPECT_EQ(frustum.getOrigin().x(), 0 * scale);
    EXPECT_EQ(frustum.getOrigin().y(), 0 * scale);
    EXPECT_EQ(frustum.getOrigin().z(), 0 * scale);

    EXPECT_EQ(frustum.getFarTopLeft().x(), 10 * scale);
    EXPECT_EQ(frustum.getFarTopLeft().y(), 1 * scale);
    EXPECT_EQ(frustum.getFarTopLeft().z(), 1 * scale);

    EXPECT_EQ(frustum.getFarTopRight().x(), 10 * scale);
    EXPECT_EQ(frustum.getFarTopRight().y(), -1 * scale);
    EXPECT_EQ(frustum.getFarTopRight().z(), 1 * scale);

    EXPECT_EQ(frustum.getFarBottomLeft().x(), 10 * scale);
    EXPECT_EQ(frustum.getFarBottomLeft().y(), 1 * scale);
    EXPECT_EQ(frustum.getFarBottomLeft().z(), -1 * scale);

    EXPECT_EQ(frustum.getFarBottomRight().x(), 10 * scale);
    EXPECT_EQ(frustum.getFarBottomRight().y(), -1 * scale);
    EXPECT_EQ(frustum.getFarBottomRight().z(), -1 * scale);
}


TEST(t_frustum, transformation_translation) {
    using V = rtl::Frustum3D<double>::VectorType;

    double xDist = 10;
    double depth = 1;
    double translationX = 2;
    rtl::Transformation3D<double> tf{rtl::Quaternion<double>{}, rtl::Vector3D<double>{translationX,0,0}};
    rtl::Frustum3D<double> frustum(V{0,0,0}, V{xDist,1,1}, V{xDist,-1,1}, V{xDist,1,-1}, V{xDist,-1,-1}, depth);
    double scale = ( (xDist + depth)/ xDist );
    auto transformedFrustum = frustum.transformed(tf);

    EXPECT_EQ(transformedFrustum.getOrigin().x(), translationX);
    EXPECT_EQ(transformedFrustum.getOrigin().y(), 0);
    EXPECT_EQ(transformedFrustum.getOrigin().z(), 0);

    EXPECT_EQ(transformedFrustum.getNearTopLeft().x(), xDist + translationX);
    EXPECT_EQ(transformedFrustum.getNearTopLeft().y(), 1);
    EXPECT_EQ(transformedFrustum.getNearTopLeft().z(), 1);

    EXPECT_EQ(transformedFrustum.getNearTopRight().x(), xDist + translationX);
    EXPECT_EQ(transformedFrustum.getNearTopRight().y(), -1);
    EXPECT_EQ(transformedFrustum.getNearTopRight().z(), 1);

    EXPECT_EQ(transformedFrustum.getNearBottomLeft().x(), xDist + translationX);
    EXPECT_EQ(transformedFrustum.getNearBottomLeft().y(), 1);
    EXPECT_EQ(transformedFrustum.getNearBottomLeft().z(), -1);

    EXPECT_EQ(transformedFrustum.getNearBottomRight().x(), xDist + translationX);
    EXPECT_EQ(transformedFrustum.getNearBottomRight().y(), -1);
    EXPECT_EQ(transformedFrustum.getNearBottomRight().z(), -1);

    EXPECT_EQ(transformedFrustum.getFarTopLeft().x(), (xDist * scale) + translationX);
    EXPECT_EQ(transformedFrustum.getFarTopLeft().y(), 1 * scale);
    EXPECT_EQ(transformedFrustum.getFarTopLeft().z(), 1 * scale);

    EXPECT_EQ(transformedFrustum.getFarTopRight().x(), (xDist * scale) + translationX);
    EXPECT_EQ(transformedFrustum.getFarTopRight().y(), -1 * scale);
    EXPECT_EQ(transformedFrustum.getFarTopRight().z(), 1 * scale);

    EXPECT_EQ(transformedFrustum.getFarBottomLeft().x(), (xDist * scale) + translationX);
    EXPECT_EQ(transformedFrustum.getFarBottomLeft().y(), 1 * scale);
    EXPECT_EQ(transformedFrustum.getFarBottomLeft().z(), -1 * scale);

    EXPECT_EQ(transformedFrustum.getFarBottomRight().x(), (xDist * scale) + translationX);
    EXPECT_EQ(transformedFrustum.getFarBottomRight().y(), -1 * scale);
    EXPECT_EQ(transformedFrustum.getFarBottomRight().z(), -1 * scale);
}

TEST(t_frustum, transformation_rotation) {
    using V = rtl::Frustum3D<double>::VectorType;

    double xDist = 10;
    double depth = 1;
    rtl::Transformation3D<double> tf{0,0,M_PI/2, rtl::Vector3D<double>{0,0,0}};
    rtl::Frustum3D<double> frustum(V{0,0,0}, V{xDist,1,1}, V{xDist,-1,1}, V{xDist,1,-1}, V{xDist,-1,-1}, depth);
    double scale = ( (xDist + depth)/ xDist );
    auto transformedFrustum = frustum.transformed(tf);

    EXPECT_NEAR(transformedFrustum.getOrigin().x(), 0, max_err);
    EXPECT_NEAR(transformedFrustum.getOrigin().y(), 0, max_err);
    EXPECT_NEAR(transformedFrustum.getOrigin().z(), 0, max_err);

    EXPECT_NEAR(transformedFrustum.getNearTopLeft().x(), -1, max_err);
    EXPECT_NEAR(transformedFrustum.getNearTopLeft().y(), xDist, max_err);
    EXPECT_NEAR(transformedFrustum.getNearTopLeft().z(), 1, max_err);

    EXPECT_NEAR(transformedFrustum.getNearTopRight().x(), 1, max_err);
    EXPECT_NEAR(transformedFrustum.getNearTopRight().y(), xDist, max_err);
    EXPECT_NEAR(transformedFrustum.getNearTopRight().z(), 1, max_err);

    EXPECT_NEAR(transformedFrustum.getNearBottomLeft().x(), -1, max_err);
    EXPECT_NEAR(transformedFrustum.getNearBottomLeft().y(), xDist, max_err);
    EXPECT_NEAR(transformedFrustum.getNearBottomLeft().z(), -1, max_err);

    EXPECT_NEAR(transformedFrustum.getNearBottomRight().x(), 1, max_err);
    EXPECT_NEAR(transformedFrustum.getNearBottomRight().y(), xDist, max_err);
    EXPECT_NEAR(transformedFrustum.getNearBottomRight().z(), -1, max_err);

    EXPECT_NEAR(transformedFrustum.getFarTopLeft().x(), -1 * scale, max_err);
    EXPECT_NEAR(transformedFrustum.getFarTopLeft().y(), (xDist * scale), max_err);
    EXPECT_NEAR(transformedFrustum.getFarTopLeft().z(), 1 * scale, max_err);

    EXPECT_NEAR(transformedFrustum.getFarTopRight().x(), 1 * scale, max_err);
    EXPECT_NEAR(transformedFrustum.getFarTopRight().y(), (xDist * scale), max_err);
    EXPECT_NEAR(transformedFrustum.getFarTopRight().z(), 1 * scale, max_err);

    EXPECT_NEAR(transformedFrustum.getFarBottomLeft().x(), -1 * scale, max_err);
    EXPECT_NEAR(transformedFrustum.getFarBottomLeft().y(), (xDist * scale), max_err);
    EXPECT_NEAR(transformedFrustum.getFarBottomLeft().z(), -1 * scale, max_err);

    EXPECT_NEAR(transformedFrustum.getFarBottomRight().x(), 1 * scale, max_err);
    EXPECT_NEAR(transformedFrustum.getFarBottomRight().y(), (xDist * scale), max_err);
    EXPECT_NEAR(transformedFrustum.getFarBottomRight().z(), -1 * scale, max_err);
}


TEST(t_frustum, transformation_tf) {
    using V = rtl::Frustum3D<double>::VectorType;

    double xDist = 10;
    double depth = 1;
    double translationX = 2;
    rtl::Transformation3D<double> tf{0,0,M_PI/2, rtl::Vector3D<double>{translationX,0,0}};
    rtl::Frustum3D<double> frustum(V{0,0,0}, V{xDist,1,1}, V{xDist,-1,1}, V{xDist,1,-1}, V{xDist,-1,-1}, depth);
    double scale = ( (xDist + depth)/ xDist );
    auto transformedFrustum = frustum.transformed(tf);

    EXPECT_NEAR(transformedFrustum.getOrigin().x(), 0 + translationX, max_err);
    EXPECT_NEAR(transformedFrustum.getOrigin().y(), 0, max_err);
    EXPECT_NEAR(transformedFrustum.getOrigin().z(), 0, max_err);

    EXPECT_NEAR(transformedFrustum.getNearTopLeft().x(), -1 + translationX, max_err);
    EXPECT_NEAR(transformedFrustum.getNearTopLeft().y(), xDist, max_err);
    EXPECT_NEAR(transformedFrustum.getNearTopLeft().z(), 1, max_err);

    EXPECT_NEAR(transformedFrustum.getNearTopRight().x(), 1 + translationX, max_err);
    EXPECT_NEAR(transformedFrustum.getNearTopRight().y(), xDist, max_err);
    EXPECT_NEAR(transformedFrustum.getNearTopRight().z(), 1, max_err);

    EXPECT_NEAR(transformedFrustum.getNearBottomLeft().x(), -1 + translationX, max_err);
    EXPECT_NEAR(transformedFrustum.getNearBottomLeft().y(), xDist, max_err);
    EXPECT_NEAR(transformedFrustum.getNearBottomLeft().z(), -1, max_err);

    EXPECT_NEAR(transformedFrustum.getNearBottomRight().x(), 1 + translationX, max_err);
    EXPECT_NEAR(transformedFrustum.getNearBottomRight().y(), xDist, max_err);
    EXPECT_NEAR(transformedFrustum.getNearBottomRight().z(), -1, max_err);

    EXPECT_NEAR(transformedFrustum.getFarTopLeft().x(), -1 * scale + translationX, max_err);
    EXPECT_NEAR(transformedFrustum.getFarTopLeft().y(), (xDist * scale), max_err);
    EXPECT_NEAR(transformedFrustum.getFarTopLeft().z(), 1 * scale, max_err);

    EXPECT_NEAR(transformedFrustum.getFarTopRight().x(), 1 * scale + translationX, max_err);
    EXPECT_NEAR(transformedFrustum.getFarTopRight().y(), (xDist * scale), max_err);
    EXPECT_NEAR(transformedFrustum.getFarTopRight().z(), 1 * scale, max_err);

    EXPECT_NEAR(transformedFrustum.getFarBottomLeft().x(), -1 * scale + translationX, max_err);
    EXPECT_NEAR(transformedFrustum.getFarBottomLeft().y(), (xDist * scale), max_err);
    EXPECT_NEAR(transformedFrustum.getFarBottomLeft().z(), -1 * scale, max_err);

    EXPECT_NEAR(transformedFrustum.getFarBottomRight().x(), 1 * scale + translationX, max_err);
    EXPECT_NEAR(transformedFrustum.getFarBottomRight().y(), (xDist * scale), max_err);
    EXPECT_NEAR(transformedFrustum.getFarBottomRight().z(), -1 * scale, max_err);
}

int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


