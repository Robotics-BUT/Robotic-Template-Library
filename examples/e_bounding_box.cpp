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

    auto BBx2D = rtl::BoundingBoxND<2, double>{rtl::VectorND<2, double>{0.0, 0.0},   // 2D Bounding box with margins in origin and the (1,1) point
                                              rtl::VectorND<2, double>{1.0, 1.0}};

    auto BBx3D = rtl::BoundingBoxND<3, double>{rtl::VectorND<3, double>{0.0, 0.0, 0.0},   // 3D Bounding box with margins in origin and the (1,1,1) point
                                              rtl::VectorND<3, double>{1.0, 1.0, 1.0}};


    auto pointBBx = rtl::BoundingBoxND<3, double>{rtl::VectorND<3, double>{0.0, 0.0, 0.0}}; // 3D BBx with zero size in the origin


    auto vectorInitializedBBx = rtl::BoundingBoxND<5, float>{ {    // 5D BBx that contains all the points given in the input vector argument
        rtl::VectorND<5, float>{0.0, 0.0, 0.0, 0.0, 0.0},
        rtl::VectorND<5, float>{1.0, 1.0, 1.0, 1.0, 1.0},
        rtl::VectorND<5, float>{2.0, 2.0, 2.0, 2.0, 2.0},
        rtl::VectorND<5, float>{3.0, 3.0, 3.0, 3.0, 3.0},
    }};


    /// Min Max

    auto maxPoint = vectorInitializedBBx.max(); // rtl::vector<5,float> {3, 3, 3, 3, 3}
    auto minPoint = vectorInitializedBBx.min(); // rtl::vector<5,float> {0, 0, 0, 0, 0}


    /// Enxtending BBx by points

    vectorInitializedBBx.addPoint(rtl::VectorND<5,float>{10.0, 10.0, 10.0, 10.0, 10.0}); // extends BBx if point is out of its borders
    maxPoint = vectorInitializedBBx.max(); // rtl::vector<5,float> {10, 10, 10, 10, 10}


    vectorInitializedBBx.addPoints({
        rtl::VectorND<5,float>{-1.0, -1.0, -1.0, -1.0, -1.0},
        rtl::VectorND<5,float>{20.0, 20.0, 20.0, 20.0, 20.0}});
    minPoint = vectorInitializedBBx.min(); // rtl::vector<5,float> {-1, -1, -1, -1, -1}
    maxPoint = vectorInitializedBBx.max(); // rtl::vector<5,float> {20, 20, 20, 20, 20}


    /// Extend BBx by BBx

    auto large3DBBx = rtl::BoundingBoxND<3, float>{ {
        rtl::VectorND<3, float>{-10.0, -10.0, -10.0},
        rtl::VectorND<3, float>{10.0, 10.0, 10.0},
    }};
    auto small3DBBx = rtl::BoundingBoxND<3, float>{ {
        rtl::VectorND<3, float>{-1.0, -1.0, -1.0},
        rtl::VectorND<3, float>{1.0, 1.0, 1.0}
    }};
    small3DBBx.addBoundingBox(large3DBBx);

    auto minPoint3D = small3DBBx.min(); // rtl::vector<3,float> {-10, -10, -10}
    auto maxPoint3D = small3DBBx.max(); // rtl::vector<3,float> {10, 10, 10}


    /// Intersection

    auto bbx2D_1 = rtl::BoundingBoxND<2, double>{rtl::VectorND<2, double>{0.0, 0.0},
                                                 rtl::VectorND<2, double>{2.0, 2.0}};
    auto bbx2D_2 = rtl::BoundingBoxND<2, double>{rtl::VectorND<2, double>{1.0, 1.0},
                                                 rtl::VectorND<2, double>{4.0, 4.0}};
    auto bool_result = bbx2D_1.intersects(bbx2D_2); // True
    auto intersectionBBx2D = rtl::BoundingBoxND<2, double>::intersection(bbx2D_2,bbx2D_2);

    auto minPoint2D = intersectionBBx2D->min(); // rtl::vector<2,double> {1, 1}
    auto maxPoint2D = intersectionBBx2D->max(); // rtl::vector<2,double> {2, 2}


    /// Others

    auto centroid = bbx2D_1.centroid(); // rtl::vector<2,double> {1, 1}
    auto volume = bbx2D_1.volume();     // 4.0

    return 0;
}