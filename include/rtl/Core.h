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

#ifndef ROBOTICTEMPLATELIBRARY_CORE_H
#define ROBOTICTEMPLATELIBRARY_CORE_H

#include "rtl/core/Utility.h"
#include "rtl/core/Constants.h"
#include "rtl/core/VectorND.h"
#include "rtl/core/LineSegmentND.h"
#include "rtl/core/Matrix.h"
#include "rtl/core/BoundingBoxND.h"
#include "rtl/core/Frustum3D.h"
#include "rtl/core/Quaternion.h"
#include "rtl/core/Polygon2D.h"
#include "rtl/core/Polygon3D.h"
#include "rtl/core/OccupancyMapND.h"

namespace rtl
{
    template<typename Element>
    using Vector2D = VectorND<2, Element>;                        //!< Partial VectorND specialization for two dimensions.
    using Vector2f = Vector2D<float>;                             //!< Full VectorND specialization for two dimensions and float elements.
    using Vector2d = Vector2D<double>;                            //!< Full VectorND specialization for two dimensions and double elements.

    template<typename Element>
    using Vector3D = VectorND<3, Element>;                        //!< Partial VectorND specialization for three dimensions.
    using Vector3f = Vector3D<float>;                             //!< Full VectorND specialization for three dimensions and float elements.
    using Vector3d = Vector3D<double>;                            //!< Full VectorND specialization for three dimensions and double elements.

    template<typename T>
    using MatrixD = Matrix<Eigen::Dynamic, Eigen::Dynamic, T>;    //!< Partial Matrix specialization for dynamic matrices.
    using Matrix22f = Matrix<2, 2, float>;                        //!< Full Matrix specialization for 2 by 2 dimensions and float elements.
    using Matrix33f = Matrix<3, 3, float>;                        //!< Full Matrix specialization for 3 by 3 dimensions and float elements.
    using Matrix22d = Matrix<2, 2, double>;                       //!< Full Matrix specialization for 2 by 2 dimensions and double elements.
    using Matrix33d = Matrix<3, 3, double>;                       //!< Full Matrix specialization for 3 by 3 dimensions and double elements.

    template<typename Element>
    using LineSegment2D = LineSegmentND<2, Element>;              //!< Partial LineSegmentND specialization for two dimensions.
    using LineSegment2f = LineSegment2D<float>;                   //!< Full LineSegmentND specialization for two dimensions and float elements.
    using LineSegment2d = LineSegment2D<double>;                  //!< Full LineSegmentND specialization for two dimensions and double elements.

    template<typename Element>
    using LineSegment3D = LineSegmentND<3, Element>;              //!< Partial LineSegmentND specialization for three dimensions.
    using LineSegment3f = LineSegment3D<float>;                   //!< Full LineSegmentND specialization for three dimensions and float elements.
    using LineSegment3d = LineSegment3D<double>;                  //!< Full LineSegmentND specialization for three dimensions and double elements.

    template<typename Element>
    using BoundingBox2D = BoundingBoxND<2, Element>;              //!< Partial BoundingBoxND specialization for two dimensions.
    using BoundingBox2f = BoundingBoxND<2, float>;                //!< Full BoundingBoxND specialization for two dimensions and float elements.
    using BoundingBox2d = BoundingBoxND<2, double>;               //!< Full BoundingBoxND specialization for two dimensions and double elements.

    template<typename Element>
    using BoundingBox3D = BoundingBoxND<3, Element>;              //!< Partial BoundingBoxND specialization for three dimensions.
    using BoundingBox3f = BoundingBoxND<3, float>;                //!< Full BoundingBoxND specialization for three dimensions and float elements.
    using BoundingBox3d = BoundingBoxND<3, double>;               //!< Full BoundingBoxND specialization for three dimensions and double elements.

    using Frustum3f = Frustum3D<float>;                           //!< Full Frustum3D specialization for float elements.
    using Frustum3d = Frustum3D<double>;                          //!< Full Frustum3D specialization for double elements.

    using Quaternionf = Quaternion<float>;                        //!< Full Quaternion specialization for float elements.
    using Quaterniond = Quaternion<double>;                       //!< Full Quaternion specialization for double elements.

    using Polygon2Df = Polygon2D<float>;                          //!< Full Polygon2D specialization for two dimensions and float elements.
    using Polygon2Dd = Polygon2D<double>;                         //!< Full Polygon2D specialization for two dimensions and double elements.

    using Polygon3Df = Polygon3D<float>;                          //!< Full Polygon3D specialization for three dimensions and float elements.
    using Polygon3Dd = Polygon3D<double>;                         //!< Full Polygon3D specialization for three dimensions and double elements.

    template<typename Element>
    using Occupancy2D = OccupancyMapND<2, Element>;                 //!< Partial OccupancyMap specialization for two dimensions
    using Occupancy2Df = OccupancyMapND<2, float>;                  //!< Full OccupancyMap specialization for two dimensions and float elements.
    using Occupancy2Dd = OccupancyMapND<2, double>;                 //!< Full OccupancyMap specialization for two dimensions and double elements.

    template<typename Element>
    using Occupancy3D = OccupancyMapND<3, Element>;                 //!< Partial OccupancyMap specialization for three dimensions
    using Occupancy3Df = OccupancyMapND<3, float>;                  //!< Full OccupancyMap specialization for three dimensions and float elements.
    using Occupancy3Dd = OccupancyMapND<3, double>;                 //!< Full OccupancyMap specialization for three dimensions and double elements.
}

#endif //ROBOTICTEMPLATELIBRARY_CORE_H
