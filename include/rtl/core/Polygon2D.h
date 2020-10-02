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

#ifndef ROBOTICTEMPLATELIBRARY_POLYGON2D_H
#define ROBOTICTEMPLATELIBRARY_POLYGON2D_H

#include <vector>

#include "rtl/core/VectorND.h"

namespace rtl
{
    template<int, typename>
    class TranslationND;

    template<int, typename>
    class RotationND;

    template<int, typename>
    class RigidTfND;

    //! Two dimensional polygon class.
    /*!
     * For now, it only aggregates points in std::vector and makes them available in a unified way with Polygon3D template.
     * Extensions such as intersection with another polygon etc. are planned in the future.
     * @tparam Element base type of underlying data.
     */
    template<typename Element>
    class Polygon2D
    {
    public:
        typedef Element ElementType;                    //!< Base type of underlying data.
        typedef VectorND<2, ElementType> VectorType;    //!< VectorND specialization for internal data.

        //! Default constructor. The polygon contains no points.
        Polygon2D() = default;

        //! Default destructor.
        ~Polygon2D() = default;

        //! Returns translated copy of the polygon.
        /*!
         * @param tr the translation to be applied.
         * @return new polygon after translation.
         */
        Polygon2D<ElementType> transformed(const TranslationND<2, Element> &tr) const
        {
            Polygon2D<ElementType> ret;
            for (const auto &p : int_pts)
                ret.addPointDirect(tr(p));
            return ret;
        }

        //! Translates *this polygon in-place.
        /*!
         *
         * @param tr the translation to be applied.
         */
        void transform(const TranslationND<2, Element> &tr)
        {
            for (auto &p : int_pts)
                p.transform(tr);
        }

        //! Returns rotated copy of the polygon.
        /*!
         * @param rot the rotation to be applied.
         * @return new polygon after rotation.
         */
        Polygon2D<ElementType> transformed(const RotationND<2, Element> &rot) const
        {
            Polygon2D<ElementType> ret;
            for (const auto &p : int_pts)
                ret.addPointDirect(rot(p));
            return ret;
        }

        //! Rotates *this polygon in-place.
        /*!
         *
         * @param rot the rotation to be applied.
         */
        void transform(const RotationND<2, Element> &rot)
        {
            for (auto &p : int_pts)
                p.transform(rot);
        }

        //! Returns transformed copy of the polygon.
        /*!
         * @param tf the transformation to be applied.
         * @return new polygon after transformation.
         */
        Polygon2D<ElementType> transformed(const RigidTfND<2, Element> &tf) const
        {
            Polygon2D<ElementType> ret;
            for (const auto &p : int_pts)
                ret.addPointDirect(tf(p));
            return ret;
        }

        //! Transforms *this polygon in-place.
        /*!
         *
         * @param tf the transformation to be applied.
         */
        void transform(const RigidTfND<2, Element> &tf)
        {
            for (auto &p : int_pts)
                p.transform(tf);
        }

        //! Read only access to the vertices.
        [[nodiscard]] const std::vector<VectorType>& points() const { return int_pts; }

        //! Reservation of the internal storage.
        /*!
         * Reallocates internal std::vector to carry \p cnt vertices.
         * @param cnt number of vertices the polygon should b able to store.
         */
        void reservePoints(size_t cnt)
        {
            int_pts.reserve(cnt);
        }

        //! Adds another vertex to the buffer.
        /*!
         *
         * @param point the new vertex of the polygon.
         */
        void addPoint(VectorType point)
        {
            int_pts.emplace_back(point);
        }

        //! Adds vertices from an iterable object using iterators.
        /*!
         * Traverses iterators pointing to \a VectorType vertices by incrementation of \p beg until the \p end is reached and adds them to the buffer.
         * @tparam Iter iterator type.
         * @param beg begin iterator.
         * @param end end (one behind, as usual) iterator.
         */
        template <class Iter>
        void addPoints(Iter beg, Iter end)
        {
            static_assert(std::is_constructible<VectorType, typename Iter::value_type>::value, "Invalid iterator value_type.");
            for (auto it = beg; it != end; it++)
                addPoint(*it);
        }

        //! Dimensionality of the polygon.
        static constexpr int dimensionality() { return 2; }

    private:
        std::vector<VectorType> int_pts;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_POLYGON2D_H
