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

#ifndef ROBOTICTEMPLATELIBRARY_VECT_PRECSUMS_H
#define ROBOTICTEMPLATELIBRARY_VECT_PRECSUMS_H

#include <eigen3/Eigen/Dense>

#include "rtl/core/VectorND.h"

namespace rtl
{
    //! Internally used structure storing names precomputed sums in 2D.
    struct PrecSums2DNames { enum {cx = 0, cy, cx2, cy2, cxy, cn}; };

    //! Internally used structure storing names precomputed sums in 3D.
    struct PrecSums3DNames { enum {cx = 0, cy, cz, cx2, cy2, cz2, cxy, cyz, czx, cn}; };

    //! Base class for precomputed sums.
    /*!
     * Common operations for all precomputed sums.
     * @tparam Compute type for precise computations.
     * @tparam Derived derived precomputed sums implementation (CRTP).
     * @tparam Names structure with enumerator naming of the sums.
     */
    template <typename Compute, class Derived, class Names>
    struct PrecSumsBase : public Names
    {
        typedef Compute ComputeType;                                    //!< Type for precise computations.
        typedef Eigen::Array<ComputeType, 1, Names::cn + 1> EigenType;  //!< Underlying Eigen type.

        //! Sum of precomputed sums.
        /*!
         * Allows to merge information arbitrarily large sets of points in constant time.
         * @param pc precomputed sums to be added to *this.
         * @return new precomputed sums representing the sum of \p pc and *this.
         */
        Derived operator+(const Derived &pc) const { return Derived(sums + pc.sums); }

        //! In-place addition of precomputed sums.
        /*!
         * Allows to add information from arbitrarily large set of points to *this in constant time.
         * @param pc precomputed sums to be added to *this.
         * @return reference to *this.
         */
        Derived& operator+=(const Derived &pc) { sums += pc.sums; return static_cast<Derived&>(*this); }

        //! Subtraction of precomputed sums.
        /*!
         * Allows to remove information contained in a subset of points in constant time.
         * @param pc precomputed sums to be subtracted from *this.
         * @return new precomputed sums representing the subtraction of \p pc from *this.
         */
        Derived operator-(const Derived &pc) const { return Derived(sums - pc.sums); }

        //! In-place subtraction of precomputed sums.
        /*!
         * Allows to remove information contained in a subset of points in constant time.
         * @param pc precomputed sums to be subtracted from *this.
         * @return reference to *this.
         */
        Derived& operator-=(const Derived &pc) { sums -= pc.sums; return static_cast<Derived&>(*this); }

        //! In-place divides all sums by number of points they represent.
        void average()
        {
            ComputeType rec = (ComputeType)1 / sums(0, Names::cn);
            sums *= rec;
        }

        //! Returns sums in *this divided by number of points they represent.
        /*!
         *
         * @return new precomputed sums representing averaged *this.
         */
        Derived averaged() const { return Derived(sums).average(); }

        //! Sets all sums to zero.
        void setZero() { sums.setZero(); }

        //! Returns number of precomputed sums.
        [[nodiscard]] static constexpr size_t sumNr() { return Names::cn; }

        EigenType sums;

    protected:
        PrecSumsBase() = default;
        explicit PrecSumsBase(EigenType &&raw_sums) : sums(raw_sums) {}
        ~PrecSumsBase() = default;
    };

    //! Precomputed sums for 2D line approximations.
    /*!
     *
     * @tparam Compute type for precise computations.
     */
    template <typename Compute>
    struct PrecSums2D : public PrecSumsBase<Compute, PrecSums2D<Compute>, PrecSums2DNames>
    {
        typedef PrecSumsBase<Compute, PrecSums2D<Compute>, PrecSums2DNames> BaseType;   //!< Specialization of PrecSumsBase.
        typedef typename BaseType::ComputeType ComputeType;                             //!< Type for precise computations.
        typedef typename BaseType::EigenType EigenType;                                 //!< Underlying Eigen type.

        using BaseType::cx, BaseType::cy, BaseType::cx2, BaseType::cy2, BaseType::cxy, BaseType::cn;

        //! Default constructor. The sums are not initialized.
        PrecSums2D() = default;

        //! Move constructor.
        explicit PrecSums2D(EigenType &&raw_sums) { BaseType::sums = std::move(raw_sums); }

        //! Construct the sums object from separate values.
        /*!
         *
         * @param sum_x sum of \a x coordinates.
         * @param sum_y sum of \a y coordinates.
         * @param sum_x2 sum of squared \a x coordinates.
         * @param sum_y2 sum of squared \a y coordinates.
         * @param sum_xy sum of product of \a x and \a y coordinates.
         * @param count number of points.
         */
        PrecSums2D(ComputeType sum_x, ComputeType sum_y, ComputeType sum_x2, ComputeType sum_y2, ComputeType sum_xy, size_t count)
        {
            BaseType::sums[cx] = sum_x;
            BaseType::sums[cy] = sum_y;
            BaseType::sums[cx2] = sum_x2;
            BaseType::sums[cy2] = sum_y2;
            BaseType::sums[cxy] = sum_xy;
            BaseType::sums[cn] = static_cast<ComputeType>(count);
        }

        //! Precomputed sums for a single point.
        /*!
         *
         * @param v the point.
         */
        explicit PrecSums2D(const VectorND<2, ComputeType> &v) : PrecSums2D(v[cx], v[cy], v[cx] * v[cx], v[cy] * v[cy], v[cx] * v[cy], 1.0) {}

        //! Returns sum of \a x coordinates.
        /*!
         *
         * @return sum of \a x coordinates.
         */
        ComputeType sx() const { return BaseType::sums[cx]; }

        //! Returns sum of \a y coordinates.
        /*!
         *
         * @return sum of \a y coordinates.
         */
        ComputeType sy() const { return BaseType::sums[cy]; }

        //! Returns sum of squared \a x coordinates.
        /*!
         *
         * @return sum of squared \a x coordinates.
         */
        ComputeType sx2() const { return BaseType::sums[cx2]; }

        //! Returns sum of squared \a y coordinates.
        /*!
         *
         * @return sum of squared \a y coordinates.
         */
        ComputeType sy2() const { return BaseType::sums[cy2]; }

        //! Returns sum of product of \a x and \a y coordinates.
        /*!
         *
         * @return sum of product of \a x and \a y coordinates.
         */
        ComputeType sxy() const { return BaseType::sums[cxy]; }

        //! Returns number of summed points.
        /*!
         *
         * @return number of summed.
         */
        ComputeType cnt() const { return BaseType::sums[cn]; }
    };

    //! Precomputed sums for 3D line and plane approximations.
    /*!
     *
     * @tparam Compute type for precise computations.
     */
    template <typename Compute>
    struct PrecSums3D : public PrecSumsBase<Compute, PrecSums3D<Compute>, PrecSums3DNames>
    {
        typedef PrecSumsBase<Compute, PrecSums3D<Compute>, PrecSums3DNames> BaseType;   //!< Specialization of PrecSumsBase.
        typedef typename BaseType::ComputeType ComputeType;                             //!< Type for precise computations.
        typedef typename BaseType::EigenType EigenType;                                 //!< Underlying Eigen type.

        using BaseType::cx, BaseType::cy, BaseType::cz, BaseType::cx2, BaseType::cy2, BaseType::cz2, BaseType::cxy, BaseType::cyz, BaseType::czx, BaseType::cn;

        //! Default constructor. The sums are not initialized.
        PrecSums3D() = default;

        //! Move constructor.
        explicit PrecSums3D(EigenType &&raw_sums) { BaseType::sums = std::move(raw_sums); }

        //! Construct the sums object from separate values.
        /*!
         *
         * @param sum_x sum of \a x coordinates.
         * @param sum_y sum of \a y coordinates.
         * @param sum_z sum of \a z coordinates.
         * @param sum_x2 sum of squared \a x coordinates.
         * @param sum_y2 sum of squared \a y coordinates.
         * @param sum_z2 sum of squared \a z coordinates.
         * @param sum_xy sum of product of \a x and \a y coordinates.
         * @param sum_yz sum of product of \a y and \a z coordinates.
         * @param sum_zx sum of product of \a z and \a x coordinates.
         * @param count number of points.
         */
        PrecSums3D(ComputeType sum_x, ComputeType sum_y, ComputeType sum_z, ComputeType sum_x2, ComputeType sum_y2, ComputeType sum_z2, ComputeType sum_xy, ComputeType sum_yz, ComputeType sum_zx, size_t count)
        {
            BaseType::sums[cx] = sum_x;
            BaseType::sums[cy] = sum_y;
            BaseType::sums[cz] = sum_z;
            BaseType::sums[cx2] = sum_x2;
            BaseType::sums[cy2] = sum_y2;
            BaseType::sums[cz2] = sum_z2;
            BaseType::sums[cxy] = sum_xy;
            BaseType::sums[cyz] = sum_yz;
            BaseType::sums[czx] = sum_zx;
            BaseType::sums[cn] = static_cast<ComputeType>(count);
        }

        //! Precomputed sums for a single point.
        /*!
         *
         * @param v the point.
         */
        explicit PrecSums3D(const VectorND<3, ComputeType> &v) : PrecSums3D(v[cx], v[cy], v[cz], v[cx] * v[cx], v[cy] * v[cy], v[cz] * v[cz], v[cx] * v[cy], v[cy] * v[cz], v[cz] * v[cx], 1.0) {}


        //! Returns sum of \a x coordinates.
        /*!
         *
         * @return sum of \a x coordinates.
         */
        ComputeType sx() const { return BaseType::sums[cx]; }

        //! Returns sum of \a y coordinates.
        /*!
         *
         * @return sum of \a y coordinates.
         */
        ComputeType sy() const { return BaseType::sums[cy]; }

        //! Returns sum of \a z coordinates.
        /*!
         *
         * @return sum of \a z coordinates.
         */
        ComputeType sz() const { return BaseType::sums[cz]; }

        //! Returns sum of squared \a x coordinates.
        /*!
         *
         * @return sum of squared \a x coordinates.
         */
        ComputeType sx2() const { return BaseType::sums[cx2]; }

        //! Returns sum of squared \a y coordinates.
        /*!
         *
         * @return sum of squared \a y coordinates.
         */
        ComputeType sy2() const { return BaseType::sums[cy2]; }

        //! Returns sum of squared \a z coordinates.
        /*!
         *
         * @return sum of squared \a z coordinates.
         */
        ComputeType sz2() const { return BaseType::sums[cz2]; }

        //! Returns sum of product of \a x and \a y coordinates.
        /*!
         *
         * @return sum of product of \a x and \a y coordinates.
         */
        ComputeType sxy() const { return BaseType::sums[cxy]; }

        //! Returns sum of product of \a y and \a z coordinates.
        /*!
         *
         * @return sum of product of \a y and \a z coordinates.
         */
        ComputeType syz() const { return BaseType::sums[cyz]; }

        //! Returns sum of product of \a z and \a x coordinates.
        /*!
         *
         * @return sum of product of \a z and \a x coordinates.
         */
        ComputeType szx() const { return BaseType::sums[czx]; }

        //! Returns number of summed points.
        /*!
         *
         * @return number of summed.
         */
        ComputeType cnt() const { return BaseType::sums[cn]; }
    };
}

#endif //ROBOTICTEMPLATELIBRARY_VECT_PRECSUMS_H
