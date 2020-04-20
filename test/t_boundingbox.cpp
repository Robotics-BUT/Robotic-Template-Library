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

rtl::BoundingBox3d getUnitBox() {
    return rtl::BoundingBox3d(rtl::VectorND<3, double>{-1, -1, -1}, rtl::VectorND<3, double>{1, 1, 1});
}

rtl::BoundingBox3d getSmallBox() {
    return rtl::BoundingBox3d(rtl::VectorND<3, double>{0, 0, 0}, rtl::VectorND<3, double>{1, 1, 1});
}

rtl::BoundingBox3d getHugeBox() {
    return rtl::BoundingBox3d(rtl::VectorND<3, double>{0, 0, 0}, rtl::VectorND<3, double>{2, 2, 2});
}

rtl::BoundingBox3d getSmallBox2() {
    return rtl::BoundingBox3d(rtl::VectorND<3, double>{2, 2, 2}, rtl::VectorND<3, double>{3, 3, 3});
}

TEST(t_boundingbox, init) {

    auto box = getUnitBox();

    EXPECT_EQ(box.min().getElement(0), -1);
    EXPECT_EQ(box.min().getElement(1), -1);
    EXPECT_EQ(box.min().getElement(2), -1);

    EXPECT_EQ(box.max().getElement(0), 1);
    EXPECT_EQ(box.max().getElement(1), 1);
    EXPECT_EQ(box.max().getElement(2), 1);
}

TEST(t_boundingbox, volume) {

    auto box = getSmallBox();

    EXPECT_EQ(box.volume(), 1);
}

TEST(t_boundingbox, intersection) {

    auto box1 = getUnitBox();
    auto box2 = getHugeBox();

    EXPECT_EQ(box1.intersects(getSmallBox2()), false);
    EXPECT_EQ(box1.intersects(box2), true);

    auto box3 = rtl::BoundingBox3d::intersection(box1, box2);

    EXPECT_EQ(box3->volume(), 1);

    EXPECT_EQ(box3->min().getElement(0), 0);
    EXPECT_EQ(box3->min().getElement(1), 0);
    EXPECT_EQ(box3->min().getElement(2), 0);

    EXPECT_EQ(box3->max().getElement(0), 1);
    EXPECT_EQ(box3->max().getElement(1), 1);
    EXPECT_EQ(box3->max().getElement(2), 1);
}

TEST(t_boundingbox, add_point) {

    auto box = getSmallBox();
    box.addPoint(rtl::VectorND<3,double>{2, 2, 2});

    EXPECT_EQ(box.min().getElement(0), 0);
    EXPECT_EQ(box.min().getElement(1), 0);
    EXPECT_EQ(box.min().getElement(2), 0);

    EXPECT_EQ(box.max().getElement(0), 2);
    EXPECT_EQ(box.max().getElement(1), 2);
    EXPECT_EQ(box.max().getElement(2), 2);
}

TEST(t_boundingbox, iou) {

    auto box1 = getUnitBox();
    auto box2 = getHugeBox();

    EXPECT_NEAR(box1.intersectionOverUnion(box2), 1.0/15.0, 0.0001);
}

TEST(t_boundingbox, transformation) {

    auto box = getUnitBox();
    double a = rtl::C_PI / 4.0;
    rtl::Transformation3d tr(a, rtl::Vector3d::baseX(), rtl::Vector3d());
    auto box_tr = box.transformed(tr);

    EXPECT_EQ(box.min().getElement(0), box_tr.min().getElement(0));
    EXPECT_NEAR(box_tr.min().getElement(1), -rtl::C_SQRT2,0.0001);
    EXPECT_NEAR(box_tr.min().getElement(2), -rtl::C_SQRT2,0.0001);

    EXPECT_EQ(box.max().getElement(0), box_tr.max().getElement(0));
    EXPECT_NEAR(box_tr.max().getElement(1), rtl::C_SQRT2, 0.0001);
    EXPECT_NEAR(box_tr.max().getElement(2), rtl::C_SQRT2, 0.0001);

    box.transform(tr);

    EXPECT_EQ(box.min().getElement(0), box_tr.min().getElement(0));
    EXPECT_EQ(box.min().getElement(1), box_tr.min().getElement(1));
    EXPECT_EQ(box.min().getElement(2), box_tr.min().getElement(2));

    EXPECT_EQ(box.max().getElement(0), box_tr.max().getElement(0));
    EXPECT_EQ(box.max().getElement(1), box_tr.max().getElement(1));
    EXPECT_EQ(box.max().getElement(2), box_tr.max().getElement(2));
}



int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


