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

#ifndef ROBOTICTEMPLATELIBRARY_VECT_APPROXIMATIONTLSLINE3D_H
#define ROBOTICTEMPLATELIBRARY_VECT_APPROXIMATIONTLSLINE3D_H

#include "rtl/Core.h"

#include "PrecSums.h"

namespace rtl
{
    //! Linear approximation of a set of points in 3D.
    /*!
     * Instances of ApproximationTlsLine3D work as functor taking PrecSumsType argument and computing linear approximation based on it.
     * Basic operations such as trim() to line segments or project() point are present as well.
     *
     * @tparam Element base type of stored elements.
     * @tparam Compute type for performing computations.
     */
    template<typename Element, typename Compute>
    class ApproximationTlsLine3D
    {
    public:
        typedef Element ElementType;                        //!< Base type of stored elements.
        typedef Compute ComputeType;                        //!< Base type for precise computations.
        typedef Vector3D<ElementType> VectorType;           //!< 3D vector with ElementType elements.
        typedef LineSegment3D<ElementType> ConstrainedType; //!< 3D line segment with ElementType elements.
        typedef PrecSums3D<ComputeType> PrecSumsType;       //!< 3D precomputed sums with ComputeType elements.

        //! Default constructor.
        ApproximationTlsLine3D() = default;

        //! Construction with direct approximation computation.
        /*!
         *
         * @param ps precomputed sums used to compute first approximation.
         */
        explicit ApproximationTlsLine3D(PrecSumsType ps) { operator()(ps); }

        //! Default destructor.
        ~ApproximationTlsLine3D() = default;

        //! Direction vector of the line.
        /*!
         *
         * @return unit direction vector.
         */
        [[nodiscard]] VectorType direction() const { return ld; }

        //! Point on the approximation line.
        /*!
         *
         * @return point on the approximation line.
         */
        [[nodiscard]] VectorType point() const { return lp; }

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
            Eigen::Matrix<ComputeType, 3, 3> cov_m;
            cov_m(0, 0) = ps.sx2() - ps.sx() * ps.sx();
            cov_m(1, 0) = ps.sxy() - ps.sx() * ps.sy();
            cov_m(2, 0) = ps.szx() - ps.sz() * ps.sx();
            cov_m(1, 1) = ps.sy2() - ps.sy() * ps.sy();
            cov_m(2, 1) = ps.syz() - ps.sy() * ps.sz();
            cov_m(2, 2) = ps.sz2() - ps.sz() * ps.sz();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<ComputeType, 3, 3>> solver;
            solver.computeDirect(cov_m);
            ld = VectorType(solver.eigenvectors().col(2).template cast<ElementType>());
            lp = VectorType(ps.sx(), ps.sy(), ps.sz());
            sigma2 = solver.eigenvalues()[0] + solver.eigenvalues()[1];

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
            return VectorType::vectorProjectionOnUnit(pt - lp, ld) + lp;
        }

        //! Projects two points onto the approximation line to trim it to line segment.
        /*!
         * @param beg begin point.
         * @param end end point.
         * @return constrained line - the line segment.
         */
        ConstrainedType trim(const VectorType &beg, const VectorType &end) const
        {
            VectorType int_beg, int_end;
            int_beg = VectorType::vectorProjectionOnUnit(beg - lp, ld) + lp;
            int_end = VectorType::vectorProjectionOnUnit(end - lp, ld) + lp;
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

        //! Squared error of approximation of given precomputed sums.
        /*!
         * The approximation is not fully evaluated, only computation of error is performed. This is faster alternative to full computation for cases, where only error value is required.
         * @param ps precomputed sums to be approximated.
         * @return squared error of linear approximation of \p ps.
         */
        static ElementType getErrorSquared(PrecSumsType ps)
        {
            ps.average();

            Eigen::Matrix<ComputeType, 3, 3> cov_m;
            cov_m(0, 0) = ps.sx2() - ps.sx() * ps.sx();
            cov_m(1, 0) = ps.sxy() - ps.sx() * ps.sy();
            cov_m(2, 0) = ps.szx() - ps.sz() * ps.sx();
            cov_m(1, 1) = ps.sy2() - ps.sy() * ps.sy();
            cov_m(2, 1) = ps.syz() - ps.sy() * ps.sz();
            cov_m(2, 2) = ps.sz2() - ps.sz() * ps.sz();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<ComputeType, 3, 3>> solver;
            solver.computeDirect(cov_m, Eigen::DecompositionOptions::EigenvaluesOnly);

            return solver.eigenvalues()[0] + solver.eigenvalues()[1];
        }

    private:
        VectorType lp, ld;
        ElementType sigma2{};
    };
}

#endif //ROBOTICTEMPLATELIBRARY_VECT_APPROXIMATIONTLSLINE3D_H
