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

#ifndef ROBOTICTEMPLATELIBRARY_VECT_APPROXIMATIONTLSLINE2D_H
#define ROBOTICTEMPLATELIBRARY_VECT_APPROXIMATIONTLSLINE2D_H

#include "rtl/Core.h"

#include "rtl/vect/PrecSums.h"

namespace rtl
{
    //! Linear approximation of a set of points in 2D.
    /*!
     * Instances of ApproximationTlsLine2D work as functor taking PrecSumsType argument and computing linear approximation based on it.
     * Basic operations such as trim() to line segments or project() point are present as well.
     *
     * @tparam Element base type of stored elements.
     * @tparam Compute type for performing computations.
     */
    template<typename Element, typename Compute>
    class ApproximationTlsLine2D
    {
    public:
        typedef Element ElementType;                        //!< Base type of stored elements.
        typedef Compute ComputeType;                        //!< Base type for precise computations.
        typedef Vector2D<ElementType> VectorType;           //!< 2D vector with ElementType elements.
        typedef LineSegment2D<ElementType> ConstrainedType; //!< 2D line segment with ElementType elements.
        typedef PrecSums2D<ComputeType> PrecSumsType;       //!< 2D precomputed sums with ComputeType elements.

        //! Default constructor.
        ApproximationTlsLine2D() = default;

        //! Construction with direct approximation computation.
        /*!
         *
         * @param ps precomputed sums used to compute first approximation.
         */
        explicit ApproximationTlsLine2D(PrecSumsType ps) { operator()(ps); }

        //! Default destructor.
        ~ApproximationTlsLine2D() = default;

        //! \a a coefficient from the general equation of a line in plane.
        /*!
         * The \a a coefficient from the \a ax + \a by + \a c = 0 equation. Corresponds to \a y coordinate of normal().
         * @return scalar coefficient.
         */
        [[nodiscard]] ElementType a() const { return -ld.y(); }

        //! \a b coefficient from the general equation of a line in plane.
        /*!
         * The \a b coefficient from the \a ax + \a by + \a c = 0 equation. Corresponds to a negative of a \a y coordinate of normal().
         * @return scalar coefficient.
         */
        [[nodiscard]] ElementType b() const { return ld.x(); }

        //! \a c coefficient from the general equation of a line in plane.
        /*!
         * The \a c coefficient from the \a ax + \a by + \a c = 0 equation.
         * @return scalar coefficient.
         */
        [[nodiscard]] ElementType c() const { return dist; }

        //! Direction vector of the line.
        /*!
         *
         * @return unit direction vector.
         */
        [[nodiscard]] VectorType direction() const { return ld; }

        //! Normal vector of the line.
        /*!
         * Normal is computed as the direction vector rotated by 90 degrees in counter-clockwise fashion.
         * @return unit normal vector.
         */
        [[nodiscard]] VectorType normal() const { return VectorType(-ld.y(), ld.x()); }

        //! Squared error of the approximation.
        /*!
         *
         * @return squared error.
         */
        [[nodiscard]] ElementType errSquared() const { return sigma2; }

        //! Functor call to process another precomputed sums.
        /*!
         * Computes linear approximation of given precomputed sums and stores the results.
         * @param ps precomputed sums to be used.
         * @return true on successful computation, false otherwise.
         */
        bool operator()(PrecSumsType ps)
        {
            ps.average();
            /*Eigen::Matrix<ComputeType, 2, 2> cov_m;
            cov_m(0, 0) = ps.sx2() - ps.sx() * ps.sx();
            cov_m(1, 0) = ps.sxy() - ps.sx() * ps.sy();
            cov_m(1, 1) = ps.sy2() - ps.sy() * ps.sy();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<ComputeType, 2, 2>> solver;
            solver.computeDirect(cov_m);
            VectorType ed;
            ed.setX(static_cast<ElementType>(solver.eigenvectors()(0, 1)));
            ed.setY(static_cast<ElementType>(solver.eigenvectors()(1, 1)));*/

            ElementType sx2 = ps.sx2() - ps.sx() * ps.sx();
            ElementType sxy = ps.sxy() - ps.sx() * ps.sy();
            ElementType sy2 = ps.sy2() - ps.sy() * ps.sy();

            ElementType trace_half = (sx2 + sy2) * 0.5;
            ElementType eig_part = std::sqrt(trace_half * trace_half - sx2 * sy2 + sxy * sxy);

            sigma2 = trace_half - eig_part;

            ld.setX((sx2 - sigma2));
            ld.setY((sxy));
            ld.normalize();

            dist = ld.y() * ps.sx() - ld.x() * ps.sy();
            return true;
        }

        //! Projects a point onto the line.
        /*!
         *
         * @param pt point to be projected.
         * @return projection of \p pt on *this.
         */
        VectorType project(const VectorType &pt) const
        {
            return pt - normal() * (VectorType::dotProduct(normal(), pt) + dist);
        }

        //! Projects two points onto the approximation line to trim it to line segment.
        /*!
         * @param beg begin point.
         * @param end end point.
         * @return constrained line - the line segment.
         */
        ConstrainedType trim(const VectorType &beg, const VectorType &end) const
        {
            VectorType int_beg, int_end, n = normal();
            int_beg = beg - n * (VectorType::dotProduct(n, beg) + dist);
            int_end = end - n * (VectorType::dotProduct(n, end) + dist);
            return ConstrainedType(int_beg, int_end);
        }

        //! Trims the approximation line to a line segment using iterator-defined set of points.
        /*!
         * Function with this signature is common to all kinds of approximation and is suitable for generic code.
         * Projection of the first point from the interval given by \p beg is used as begin of the line segment and projection the point before \p end gives the end-point.
         * @tparam Iter iterator type.
         * @param beg begin iterator.
         * @param end end (one behind, as usual) iterator.
         * @return constrained line - the line segment.
         */
        template <class Iter>
        ConstrainedType trim(Iter beg, Iter end) const
        {
            static_assert(std::is_constructible<VectorType, typename Iter::value_type>::value, "Invalid iterator value_type.");
            return trim(*beg, *(--end));
        }

        //! Crossing of two approximations.
        /*!
         * If the intersection does not exist, \p crossing is not modified.
         * @param l1 the first approximation.
         * @param l2 the second approximation.
         * @param crossing return parameter specifying the intersection.
         * @return true if intersection was found, false otherwise.
         */
        static bool getCrossing(const ApproximationTlsLine2D<ElementType, ComputeType> &l1, const ApproximationTlsLine2D<ElementType, ComputeType> &l2, VectorType &crossing)
        {
            ElementType det = l1.a() * l2.b() - l1.b() * l2.a();
            if (det == 0.0)
                return false;
            crossing.setX(-l1.c() * l2.b() + l2.c() * l1.b());
            crossing.setY(l2.a() * l1.c()-l1.a() * l2.c());
            crossing /= det;
            return true;
        }

        //! Squared error of approximation of given precomputed sums.
        /*!
         * The approximation is not fully evaluated, only computation of error is performed. This is faster alternative to full computation for cases, where only error value is required.
         * @param ps precomputed sums to be approximated.
         * @return squared error of linear approximation of \p ps.
         */
        static ElementType getErrorSquared(PrecSumsType ps)
        {
            ps.average();

            ElementType sx2 = ps.sx2() - ps.sx() * ps.sx();
            ElementType sxy = ps.sxy() - ps.sx() * ps.sy();
            ElementType sy2 = ps.sy2() - ps.sy() * ps.sy();
            ElementType trace_half = (sx2 + sy2) * 0.5;

            return trace_half - std::sqrt(trace_half * trace_half - sx2 * sy2 + sxy * sxy);
        }

    private:
        VectorType ld;
        ElementType dist{}, sigma2{};
    };
}

#endif //ROBOTICTEMPLATELIBRARY_VECT_APPROXIMATIONTLSLINE2D_H
