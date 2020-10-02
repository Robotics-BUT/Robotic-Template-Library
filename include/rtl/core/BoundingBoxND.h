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

#ifndef ROBOTICTEMPLATELIBRARY_BOUNDINGBOXND_H
#define ROBOTICTEMPLATELIBRARY_BOUNDINGBOXND_H

#include <vector>
#include <exception>
#include <memory>
#include "rtl/core/VectorND.h"

namespace rtl
{
    template<int, typename>
    class TranslationND;

    template<int, typename>
    class RotationND;

    template<int, typename>
    class RigidTfND;

    //! Axis aligned bounding box - implementation for N-dimensional space.
    /*! Generic implementation of an axis aligned bounding box in N-dimensional space scpecified by two vertices. The min() vertex represents the lower bound in all dimensions and the
     * max() vertex represents the corresponding upper one. This feature is maintained reliably for BoundingBox operations. Note that default constructor of BoundingBoxND is deleted, because any
     * uninitialized values would affect its shape when new points are added. Also note, that since it is axix aligned, many transformations (e.g. rotation) will cause the bounding
     * box to grow to reliably cover the volume spanned before the transformation was applied. This means, thant in general \f$BB \neq T^{-1}(T(BB))\f$. Also, the bounding box cannot
     * be shrank, because it does not keep track of the objects it covers.
     *
     * @tparam dim dimensional space the bounding box will belong to.
     * @tparam Element type for underlying vectors.
     */

    template<int dim, typename Element>
    class BoundingBoxND
    {
    public:
        typedef Element ElementType;                    //!< Base data type.
        typedef VectorND<dim, ElementType> VectorType;  //!< Vector type of the bounding box vertices and the input/query interface.

        //! The default constructor is deleted, because any uninitialized values would affect its shape when new points are added.
        BoundingBoxND() = delete;

        //! Copy constructor.
        /*!
         * @param bb bounding box to be copied.
         */
        BoundingBoxND(const BoundingBoxND &bb) : b_min(bb.b_min), b_max(bb.b_max) {}

        //! Initialization with a single point.
        /*!
         * In that case min() = max() and volume() returns zero.
         * @param v the initializing point.
         */
        explicit BoundingBoxND(const VectorType &v) : b_min(v), b_max(v) {}

        //! Two point constructor for non-zero volume bounding boxes.
        /*!
         * Any two points will suffice, separation of lower and upper bounds is performed automatically.
         * @param v1 first point.
         * @param v2 second point.
         */
        BoundingBoxND(const VectorType &v1, const VectorType &v2) : b_min(minPoint(v1, v2)), b_max(maxPoint(v1, v2)) {}

        //! Construction from std::vector of points.
        /*!
         * The resulting bounding box is the tightest axis aligned volume containing all points. If the vector is empty,
         * a std::invalid_argument exception is thrown.
         * @param vects points to be used for construction.
         */
        explicit BoundingBoxND(const std::vector<VectorType> &vects)
        {
            if (vects.empty())
                throw std::invalid_argument("Empty vector of points supplied to BoundingBoxND constructor.");
            b_min = b_max = vects[0];
            for (size_t i = 1; i < vects.size(); i++)
            {
                b_min = minPoint(b_min, vects[i]);
                b_max = maxPoint(b_max, vects[i]);
            }
        }

        [[nodiscard]] VectorType min() const { return b_min; }  //!< Vector representing lower bounds of the bounding box in all dimensions.
        [[nodiscard]] VectorType max() const { return b_max; }  //!< Vector representing upper bounds of the bounding box in all dimensions.

        //! Adjusts the bounding box to cover point \p p.
        /*!
         * If \p p lies within the bounding box margins, nothing happens. If the point is outside, the bounding box is expanded accordingly.
         * @param p point to be examined.
         */
        void addPoint(const VectorType &p)
        {
            b_min = minPoint(b_min, p);
            b_max = maxPoint(b_max, p);
        }

        //! Adjusts the bounding box to cover all points in \p pts.
        /*!
         * If a point from \p pts lies within the bounding box margins, nothing happens. If it is outside, the bounding box is expanded accordingly.
         * @param pts points to be examined.
         */
        void addPoints(const std::vector<VectorType> &pts)
        {
            for (const VectorType &p : pts)
            {
                b_min = minPoint(b_min, p);
                b_max = maxPoint(b_max, p);
            }
        }

        //! Adjusts the bounding box to cover another bounding box \p bb as well.
        /*!
         * If \p bb is entirely covered then nothing happens. Otherwise the bounding box is expanded accordingly.
         * @param bb bounding box to be examined.
         */
        void addBoundingBox(const BoundingBoxND &bb)
        {
            b_min = minPoint(b_min, bb.b_min);
            b_max = maxPoint(b_max, bb.b_max);
        }

        //! Tests if there is any intersection with \p bb or not.
        /*!
         * Strict inequality is user for testing, so touching bounding boxes do not return true.
         * @param bb bounding box tested for intersection.
         * @return true if there is non-zero-volume intersection, false otherwise.
         */
        bool intersects(const BoundingBoxND &bb) const
        {
            for (size_t i = 0; i < dim; i++)
                if (bb.b_max[i] < b_min[i] || bb.b_min[i] > b_max[i])
                    return false;
            return true;
        }

        //! From min() and max() vertices generates a std::vector of all vertices of the bounding box and applies \p func on them.
        /*!
         * If called with default argument, no processing is applied to the vertices and std::vector containing them is returned.
         * @tparam T type of invokable object (function, functor, lambda, ...) with VectorType parameter.
         * @param func the actual invokable object, identity by default.
         * @return all vertices of the bounding box processed by \p func.
         */
        template<typename T>
        std::vector<typename std::invoke_result_t<T, VectorType>> allVertices(T &&func = identityFunc) const
        {
            std::vector<typename std::invoke_result_t<T, VectorType>> vertices;
            vertices.reserve(1u << dim);
            VectorType v;
            for (size_t i = 0; i < (1u << dim); i++)
            {
                for (size_t j = 0; j < dim; j++)
                    v[j] = (i & (1u << j)) ? b_min[j] : b_max[j];
                vertices.push_back(func(v));
            }
            return vertices;
        }

        //! Returns translated copy of the bounding box.
        /*!
         * @param tr the translation to be applied.
         * @return new bounding box after translation.
         */
        BoundingBoxND<dim, Element> transformed(const TranslationND<dim, Element> &tr) const
        {
            BoundingBoxND<dim, Element> ret(*this);
            ret.b_min += tr.trVec();
            ret.b_max += tr.trVec();
            return ret;
        }

        //! Translates *this bounding box in-place.
        /*!
         *
         * @param tr the translation to be applied.
         */
        void transform(const TranslationND<dim, Element> &tr)
        {
            b_min += tr.trVec();
            b_max += tr.trVec();
        }

        //! Returns rotated copy of the bounding box.
        /*!
         * @param rot the rotation to be applied.
         * @return new bounding box after rotation.
         */
        BoundingBoxND<dim, Element> transformed(const RotationND<dim, Element> &rot) const
        {
            return BoundingBoxND<dim, Element>(allVertices(rot));
        }

        //! Rotates *this bounding box in-place.
        /*!
         *
         * @param rot the rotation to be applied.
         */
        void transform(const RotationND<dim, Element> &rot)
        {
            auto tmp = this->transformed(rot);
            b_min = tmp.b_min;
            b_max = tmp.b_max;
        }

        //! Returns transformed copy of the bounding box.
        /*!
         * @param tf the transformation to be applied.
         * @return new bounding box after transformation.
         */
        BoundingBoxND<dim, Element> transformed(const RigidTfND<dim, Element> &tf) const
        {
            return BoundingBoxND<dim, Element>(allVertices(tf));
        }

        //! Transforms *this bounding box in-place.
        /*!
         *
         * @param tf the transformation to be applied.
         */
        void transform(const RigidTfND<dim, Element> &tf)
        {
            auto tmp = this->transformed(tf);
            b_min = tmp.b_min;
            b_max = tmp.b_max;
        }

        //! Computes hyper-volume spanned by min() and max() vectors.
        /*!
         * @return hyper-volume of the bounding box.
         */
        Element volume() const
        {
            Element vol = 1;
            for (size_t i = 0; i < dim; i++)
                vol *= b_max[i] - b_min[i];
            return vol;
        }

        //! Computes central point of the bounding box.
        /*!
         * @return centroid of the bounding box.
         */
        VectorType centroid() const
        {
            return (b_min + b_max) / 2;
        }

        //! Ratio of the volumes of intersection of the bounding boxes over their union.
        /*!
         * Parameter ranging from 0 for no overlap to 1 for perfect overlay.
         * @param bb bounding box to test against.
         * @return the ratio.
         */
        Element intersectionOverUnion(const BoundingBoxND &bb) const
        {
            auto intersected = BoundingBoxND<dim, ElementType>::intersection(*this, bb);
            if(intersected)
            {
                Element intersection_vol = intersected->volume();
                Element union_vol = volume() + bb.volume() - intersection_vol;
                return intersection_vol / union_vol;
            }
            return 0.0;
        }

        //! Computes intersection of two bounding boxes if it exists.
        /*!
         * Currently produces std::unique_ptr to deal with cases, where no intersection exists. Ths might change in the future.
         * @param bb1 first bounding box to be examined.
         * @param bb2 second bounding box to be examined.
         * @return intersection of \p bb1 and \p bb2 if it exists, nullptr otherwise.
         */
        static std::unique_ptr<BoundingBoxND<dim, ElementType>> intersection(const BoundingBoxND<dim, ElementType> &bb1, const BoundingBoxND<dim, ElementType> &bb2)
        {
            VectorType t_min(bb1.b_min.data().array().max(bb2.b_min.data().array()).matrix());
            VectorType t_max(bb1.b_max.data().array().min(bb2.b_max.data().array()).matrix());
            if ((t_min.data().array() < t_max.data().array()).all())
                return std::make_unique<BoundingBoxND<dim, ElementType>>(t_min, t_max);
            else
                return std::unique_ptr<BoundingBoxND<dim, ElementType>>(nullptr);
        }

        //! Dimensionality of the bounding box.
        static constexpr int dimensionality() { return dim; }

    private:
        static VectorType identityFunc(VectorType &&v) { return v; }

        VectorType minPoint(const VectorType &v1, const VectorType &v2) const
        {
            return VectorType(v1.data().array().min(v2.data().array()).matrix());
        }

        VectorType maxPoint(const VectorType &v1, const VectorType &v2) const
        {
            return VectorType(v1.data().array().max(v2.data().array()).matrix());
        }

        VectorType b_min, b_max;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_BOUNDINGBOXND_H
