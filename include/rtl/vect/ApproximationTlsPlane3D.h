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

#ifndef ROBOTICTEMPLATELIBRARY_VECT_APPROXIMATIONTLSPLANE3D_H
#define ROBOTICTEMPLATELIBRARY_VECT_APPROXIMATIONTLSPLANE3D_H

#include "rtl/Core.h"
#include "rtl/vect/PrecSums.h"

namespace rtl
{
    //! Planar approximation of a set of points in 3D.
    /*!
     * Instances of ApproximationTlsPlane3D work as functor taking PrecSumsType argument and computing linear approximation based on it.
     * Basic operations such as trim() to line segments or project() point are present as well.
     *
     * @tparam Element base type of stored elements.
     * @tparam Compute type for performing computations.
     */
    template<typename Element, typename Compute>
    class ApproximationTlsPlane3D
    {
    public:
        typedef Element ElementType;                    //!< Base type of stored elements.
        typedef Compute ComputeType;                    //!< Base type for precise computations.
        typedef Vector3D<ElementType> VectorType;       //!< 3D vector with ElementType elements.
        typedef Polygon3D<ElementType> ConstrainedType; //!< 3D polygon with ElementType elements.
        typedef PrecSums3D<ComputeType> PrecSumsType;   //!< 3D precomputed sums with ComputeType elements.

        //! Default constructor.
        ApproximationTlsPlane3D() = default;

        //! Construction with direct approximation computation.
        /*!
         *
         * @param ps precomputed sums used to compute first approximation.
         */
        explicit ApproximationTlsPlane3D(PrecSumsType ps) { operator()(ps); }

        //! Default destructor.
        ~ApproximationTlsPlane3D() = default;

        //! Normal of the plane.
        /*!
         *
         * @return unit normal vector.
         */
        [[nodiscard]] VectorType normal() const { return pn; }

        //! \a d coefficient from the general equation of a plane.
        /*!
         * The \a d coefficient from the \a ax + \a by + \a cz + \a d = 0 equation.
         * @return scalar coefficient.
         */
        [[nodiscard]] ElementType d() const { return pd; }

        //! Squared error of the approximation.
        /*!
         *
         * @return squared error.
         */
        [[nodiscard]] ElementType errSquared() const { return sigma2; }

        //! Functor call to process another precomputed sums.
        /*!
         * Computes planar approximation of given precomputed sums and stores the results.
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
            pn = VectorType(solver.eigenvectors().col(0).template cast<ElementType>());
            pd = VectorType::scalarProjectionOnUnit(VectorType(ps.sx(), ps.sy(), ps.sz()), pn);
            sigma2 = solver.eigenvalues()[0];

            return true;
        }

        //! Projects a point onto the plane.
        /*!
         *
         * @param pt point to be projected.
         * @return projection of \p pt on *this.
         */
        VectorType project(const VectorType &pt) const
        {
            return pn * (VectorType::scalarProjectionOnUnit(pt, pn) + pd);
        }

        //! Trims the approximation plane to a polygon using iterator-defined set of points.
        /*!
         * Function with this signature is common to all kinds of approximation and is suitable for generic code.
         * All points given by interval \p beg to \p end are projected onto *this and for an outline of the resulting polygon.
         * @tparam Iter iterator type.
         * @param beg begin iterator.
         * @param end end (one behind, as usual) iterator.
         * @param size_hint expected number of points specified by \p beg to \p end. Zero by default - no hint available.
         * @return constrained plane - the polygon.
         */
        template <class Iter>
        ConstrainedType trim(Iter beg, Iter end, size_t size_hint = 0) const
        {
            static_assert(std::is_constructible<VectorType, typename Iter::value_type>::value, "Invalid iterator value_type.");
            ConstrainedType out(pn, pd);
            if (size_hint != 0)
                out.reservePoints(size_hint);
            for (auto it = beg; it != end; it++)
                out.addPoint(*it);
            return out;
        }

        //! Trims the planar approximation by std::vector of points.
        /*!
         *
         * @param pts points to be projected onto *this forming an outline of the polygon.
         * @return constrained plane - the polygon.
         */
        ConstrainedType trim(const std::vector<VectorType> &pts) const
        {
            return trim(pts.begin(), pts.end(), pts.size());
        }

        //! Squared error of approximation of given precomputed sums.
        /*!
         * The approximation is not fully evaluated, only computation of error is performed. This is faster alternative to full computation for cases, where only error value is required.
         * @param ps precomputed sums to be approximated.
         * @return squared error of planar approximation of \p ps.
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

            return solver.eigenvalues()[0];
        }

    private:
        VectorType pn;
        ElementType pd{}, sigma2{};
    };
}


#endif //ROBOTICTEMPLATELIBRARY_VECT_APPROXIMATIONTLSPLANE3D_H
