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

#ifndef ROBOTICTEMPLATELIBRARY_POLYGON3D_H
#define ROBOTICTEMPLATELIBRARY_POLYGON3D_H

#include <vector>
#include "rtl/core/VectorND.h"

namespace rtl
{
    template<typename Element>
    class Transformation3D;

    //! Three dimensional polygon class.
    /*!
     * For now, it aggregates points in std::vector, stores data of the plane in which they lie and allow transformation by Transformation3D.
     * Extensions such as intersection with another polygon etc. are planned in the future.
     * @tparam Element base type of underlying data.
     */
    template<typename Element>
    class Polygon3D
    {
    public:
        typedef Element ElementType;                    //!< Base type of underlying data.
        typedef VectorND<3, ElementType> VectorType;    //!< VectorND specialization for internal data.

        //! Default constructor. The polygon contains no points.
        Polygon3D() = default;

        //! Construction from plane in which the polygon lies.
        /*!
         *
         * @param normal vector orthogonal to the plane of the polygon. Its length is normalized during construction.
         * @param distance along the (unit!) \p normal from origin to the plane.
         */
        Polygon3D(VectorType normal, ElementType distance) : int_dist(distance)
        {
            int_normal = normal.normalized();
        }

        //! Default destructor.
        ~Polygon3D() = default;

        //! Unit normal vector of the polygon.
        /*!
         *
         * @return unit normal vector.
         */
        [[nodiscard]] VectorType normal() const { return int_normal; }

        //! Signed distance along the normal() from origin to the plane.
        /*!
         *
         * @return scalar distance value.
         */
        [[nodiscard]] ElementType distance() const { return int_dist; }

        //! \a a coefficient from the general equation of a plane.
        /*!
         * The \a a coefficient from the \a ax + \a by + \a cz + \a d = 0 equation. Corresponds to \a x coordinate of normal().
         * @return scalar coefficient.
         */
        [[nodiscard]] ElementType a() const { return int_normal.x(); }

        //! \a b coefficient from the general equation of a plane.
        /*!
         * The \a b coefficient from the \a ax + \a by + \a cz + \a d = 0 equation. Corresponds to \a y coordinate of normal().
         * @return scalar coefficient.
         */
        [[nodiscard]] ElementType b() const { return int_normal.y(); }

        //! \a c coefficient from the general equation of a plane.
        /*!
         * The \a c coefficient from the \a ax + \a by + \a cz + \a d = 0 equation. Corresponds to \a z coordinate of normal().
         * @return scalar coefficient.
         */
        [[nodiscard]] ElementType c() const { return int_normal.z(); }

        //! \a d coefficient from the general equation of a plane.
        /*!
         * The \a d coefficient from the \a ax + \a by + \a cz + \a d = 0 equation. Corresponds to - distance().
         * @return scalar coefficient.
         */
        [[nodiscard]] ElementType d() const { return -int_dist; }

        //! Read only access to the vertices.
        [[nodiscard]] const std::vector<VectorType>& points() const { return int_pts; }

        Polygon3D<Element> transformed(const Transformation3D<Element> &tf) const
        {
            auto new_normal = tf.rotMat() * int_normal;
            ElementType  new_dist = int_dist + VectorType::dotProduct(tf.tr(), new_normal);
            Polygon3D<ElementType> ret(new_normal, new_dist);
            for (const auto &p : int_pts)
                ret.addPointDirect(tf(p));
            return ret;
        }

        void transform(const Transformation3D<Element> &tf)
        {
            int_normal.transform(tf);
            int_dist += VectorType::dotProduct(tf.tr(), int_normal);
            for (auto &p : int_pts)
                p.transform(tf);
        }

        //! Reservation of the internal storage.
        /*!
         * Reallocates internal std::vector to carry \p cnt vertices.
         * @param cnt number of vertices the polygon should b able to store.
         */
        void reservePoints(size_t cnt)
        {
            int_pts.reserve(cnt);
        }

        //! Adds projection of \p point as another vertex to the buffer.
        /*!
         * Projection of \p point to the polygon plane is performed to ensure the polygon is truly planar.
         * @param point new vertex to be added.
         */
        void addPoint(const VectorType &point)
        {
            int_pts.emplace_back(point - (VectorType::scalarProjectionOnUnit(point, int_normal) - int_dist) * int_normal);
        }

        //! Adds projections of vertices from an iterable object using iterators.
        /*!
         * Traverses iterators pointing to \a VectorType vertices by incrementation of \p beg until the \p end is reached and adds their
         * projections into the polygon plane to the internal buffer.
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

        //! Adds \p point as another vertex to the buffer.
        /*!
         * No checks, whether \p point lies in the polygon plane are performed, so it is possible to make an invalid polygon that way.
         * Use for increased speed when \p point is guaranteed to be in the polygon plane.
         * @param point new vertex to be added.
         */
        void addPointDirect(VectorType point)
        {
            int_pts.emplace_back(point);
        }

        //! Adds vertices from an iterable object using iterators.
        /*!
         * Traverses iterators pointing to \a VectorType vertices by incrementation of \p beg until the \p end is reached and adds directly
         * to the internal buffer. No checks, whether the vertices lie in the polygon plane are performed, so it is possible to make an invalid
         * polygon that way. Use for increased speed when vertices in the range \p beg to \p end are guaranteed to be in the polygon plane.
         * @tparam Iter iterator type.
         * @param beg begin iterator.
         * @param end end (one behind, as usual) iterator.
         */
        template <class Iter>
        void addPointsDirect(Iter beg, Iter end)
        {
            static_assert(std::is_constructible<VectorType, typename Iter::value_type>::value, "Invalid iterator value_type.");
            for (auto it = beg; it != end; it++)
                addPointDirect(*it);
        }

    private:
        VectorType int_normal;
        ElementType int_dist{};
        std::vector<VectorType> int_pts;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_POLYGON3D_H
