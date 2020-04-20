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

#ifndef ROBOTICTEMPLATELIBRARY_FRUSTUM3D_H
#define ROBOTICTEMPLATELIBRARY_FRUSTUM3D_H

#include "rtl/core/VectorND.h"

namespace rtl
{
    template<typename Element>
    class Transformation3D;

    //! 3D frustum representation.
    /*!
     * Fast implementation of a frustum object for storing detections from image data. The class does not perform any chcecks of data during construction, so it is possible to construct
     * an invalid frustum that way. On the other hand, the implementation is very fast and requires minimum computational overhead.
     * @tparam Element underlying type of numeric data.
     */
    template<typename Element>
    class Frustum3D
    {
    public:
        typedef Element ElementType;                //!< Base data type.
        typedef VectorND<3, Element> VectorType;    //!< Vector type of the bounding box vertices and the input/query interface.

        //! Construction from points.
        /*!
         * Constructs the frustum from its points. No valididty checks are performed.
         * @param origin virtual tip of the original "pyramid".
         * @param nearTopLeft top left corner of the near plane.
         * @param nearTopRight top right corner of the near plane.
         * @param nearBottomLeft bottom left corner of the near plane.
         * @param nearBottomRight bottom right corner of the near plane.
         * @param depth distance between near and far plane.
         */
        Frustum3D(const VectorType &origin, const VectorType &nearTopLeft, const VectorType &nearTopRight,
                  const VectorType &nearBottomLeft, const VectorType &nearBottomRight, Element depth) :
                  origin_{origin}, nearTopLeft_{nearTopLeft}, nearTopRight_{nearTopRight}, nearBottomLeft_{nearBottomLeft}, nearBottomRight_{nearBottomRight}, frustumDepth_{depth}
        {
        }

        //! Copy constructor.
        Frustum3D (const Frustum3D<Element>& f) : origin_{f.origin_}, nearTopLeft_{f.nearTopLeft_}, nearTopRight_{f.nearTopRight_},
                                                  nearBottomLeft_{f.nearBottomLeft_}, nearBottomRight_{f.nearBottomRight_}, frustumDepth_{f.frustumDepth_}
        {
        }

        //! Return origin (the virtual tip) of the frustum.
        [[nodiscard]] VectorType getOrigin() const {return origin_;}
        //! Returns top left corner of the near plane.
        [[nodiscard]] VectorType getNearTopLeft() const {return nearTopLeft_;}
        //! Returns top right corner of the near plane.
        [[nodiscard]] VectorType getNearTopRight() const {return nearTopRight_;}
        //! Returns bottom left corner of the near plane.
        [[nodiscard]] VectorType getNearBottomLeft() const {return nearBottomLeft_;}
        //! Returns bottom right corner of the near plane.
        [[nodiscard]] VectorType getNearBottomRight() const {return nearBottomRight_;}
        //! Returns top left corner of the far plane.
        [[nodiscard]] VectorType getFarTopLeft() const {return scalePoint(origin_, nearTopLeft_, ((getNearMidPoint()-origin_).length()+frustumDepth_) / (getNearMidPoint()-origin_).length());}
        //! Returns top right corner of the far plane.
        [[nodiscard]] VectorType getFarTopRight() const {return scalePoint(origin_, nearTopRight_, ((getNearMidPoint()-origin_).length()+frustumDepth_) / (getNearMidPoint()-origin_).length());}
        //! Returns bottom left corner of the far plane.
        [[nodiscard]] VectorType getFarBottomLeft() const {return scalePoint(origin_, nearBottomLeft_, ((getNearMidPoint()-origin_).length()+frustumDepth_) / (getNearMidPoint()-origin_).length());}
        //! Returns bottom right corner of the far plane.
        [[nodiscard]] VectorType getFarBottomRight() const {return scalePoint(origin_, nearBottomRight_, ((getNearMidPoint()-origin_).length()+frustumDepth_) / (getNearMidPoint()-origin_).length());}
        //! Returns central point of the near plane.
        [[nodiscard]] VectorType getNearMidPoint() const {return (nearTopLeft_+nearTopRight_+nearBottomLeft_+nearBottomRight_) / 4; }
        //! Returns distance between near and far plane.
        [[nodiscard]] ElementType getDepth() const { return frustumDepth_; }

        //! Returns transformed copy of the frustum.
        /*!
         * @param tf the transformation to be applied.
         * @return new frustum instance after transformation.
         */
        Frustum3D<Element> transformed(const Transformation3D<Element> &tf) const
        {
            return Frustum3D<Element>(tf(origin_), tf(nearTopLeft_), tf(nearTopRight_), tf(nearBottomLeft_), tf(nearBottomRight_), frustumDepth_);
        }

        //! Transforms the frustum in place.
        /*!
         * @param tf the transformation to be applied.
         */
        void transform(const Transformation3D<Element> &tf)
        {
            origin_.transform(tf);
            nearTopLeft_.transform(tf);
            nearTopRight_.transform(tf);
            nearBottomLeft_.transform(tf);
            nearBottomRight_.transform(tf);
        }

    private:
        VectorType origin_;
        VectorType nearTopLeft_;
        VectorType nearTopRight_;
        VectorType nearBottomLeft_;
        VectorType nearBottomRight_;
        Element frustumDepth_;

        static VectorType scalePoint(const VectorType& origin, const VectorType&direction, Element scale)
        {
            auto line = direction - origin;
            auto scaled = line * scale;
            auto output = origin + scaled;
            return output;
        }

    };
}

#endif //ROBOTICTEMPLATELIBRARY_FRUSTUM3D_H
