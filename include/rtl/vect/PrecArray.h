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

#ifndef ROBOTICTEMPLATELIBRARY_VECT_PRECARRAY_H
#define ROBOTICTEMPLATELIBRARY_VECT_PRECARRAY_H

#include <eigen3/Eigen/Dense>

#include "rtl/vect/PrecSums.h"

namespace rtl
{
    //! Base class for precomputed arrays.
    /*!
     * Common operations for all precomputed arrays are implemented here. Adds the first row of zeros to the array
     * making the array one row longer than the number of points used to its precomputation. This feature makes computation
     * of precomputed sums as simple as subtraction of two rows of the array in all cases.
     * @tparam Compute type for precise computations.
     * @tparam SumType precomputed sum type.
     */
    template <typename Compute, class SumType>
    struct PrecArayBase
    {
        typedef Compute ComputeType;        //!< Type for precise computations.
        typedef Eigen::Array<ComputeType, Eigen::Dynamic, SumType::sumNr()> EigenType; //!< Underlying Eigen type.

        //! Default constructor.
        PrecArayBase() = default;

        //! Default destructor.
        ~PrecArayBase() = default;

        //! Returns total size of the array including the first row of zeros.
        /*!
         *
         * @return size of the array.
         */
        [[nodiscard]] size_t size() const { return array_size; }

        //! Change size of the array.
        /*!
         * Resizes the array to take required number of points plus the initial row of zeros. size() then returns \p pts_cnt + 1.
         * @param pts_cnt number of points.
         */
        void resize(size_t pts_cnt)
        {
            pts_cnt += 1; // for initial row of zeros
            if (static_cast<size_t>(array.rows()) < pts_cnt)
            {
                array.resize(pts_cnt, Eigen::NoChange);
                array.row(0).setZero();
            }
            array_size = pts_cnt;
        }

        //! Returns precomputed sums from given row of the array.
        /*!
         *
         * @param index row index.
         * @return precomputed sums.
         */
        SumType sums(size_t index) const
        {
            SumType ret;
            ret.sums.block(0, 0, 1, SumType::sumNr()) = array.row(index);
            ret.sums(0, SumType::sumNr()) = index;
            return ret;
        }

        //! Returns precomputed sums for range of points.
        /*!
         *
         * @param beg first point of the interval of interest.
         * @param end one behind the last point of the interval of interest.
         * @return precomputed sums.
         */
        SumType sums(size_t beg, size_t end) const
        {
            SumType ret;
            ret.sums.block(0, 0, 1, SumType::sumNr()) = array.row(end) - array.row(beg);
            ret.sums(0, SumType::sumNr()) = end - beg;
            return ret;
        }

        EigenType array;
        size_t array_size{};
    };

    //! Precomputed array for 2D total least squares fitting of lines.
    /*!
     *
     * @tparam Element type for data element storage.
     * @tparam Compute type for precise computation.
     */
    template <typename Element, typename Compute>
    struct PrecArray2D : public PrecArayBase<Compute, PrecSums2D<Compute>>
    {
        typedef PrecSums2D<Compute> SumsType;                   //!< Type of precomputed sums.
        typedef PrecArayBase<Compute, SumsType> BaseType;       //!< Specialization of PrecArayBase.
        typedef Element ElementType;                            //!< Type for data element storage.
        typedef typename BaseType::ComputeType ComputeType;     //!< Type for precise computation.
        typedef typename BaseType::EigenType EigenType;         //!< Underlying Eigen type.

        //! Precompute sums for given points.
        /*!
         * Points are organized in std::vector and the function rely on fact that they are stored in correct order in continuous chunk of memory.
         * @param vec points for precomputation.
         */
        void precompute(const std::vector<rtl::Vector2D<ElementType>> &vec)
        {
            size_t vec_size = vec.size();
            BaseType::resize(vec_size);

            Eigen::Map<const Eigen::Matrix<ElementType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> map(&vec[0][0], vec_size, 2);
            BaseType::array.block(1, SumsType::cx, vec_size, 1) = map.array().col(SumsType::cx).template cast<ComputeType>();
            BaseType::array.block(1, SumsType::cy, vec_size, 1) = map.array().col(SumsType::cy).template cast<ComputeType>();
            BaseType::array.block(1, SumsType::cx2, vec_size, 1) = BaseType::array.block(1, SumsType::cx, vec_size, 1) * BaseType::array.block(1, SumsType::cx, vec_size, 1);
            BaseType::array.block(1, SumsType::cy2, vec_size, 1) = BaseType::array.block(1, SumsType::cy, vec_size, 1) * BaseType::array.block(1, SumsType::cy, vec_size, 1);
            BaseType::array.block(1, SumsType::cxy, vec_size, 1) = BaseType::array.block(1, SumsType::cx, vec_size, 1) * BaseType::array.block(1, SumsType::cy, vec_size, 1);

            for (size_t i = 1; i < vec_size + 1; i++)
                BaseType::array.row(i) += BaseType::array.row(i - 1);
        }
    };

    //! Precomputed array for 3D total least squares fitting of lines and planes.
    /*!
     *
     * @tparam Element type for data element storage.
     * @tparam Compute type for precise computation.
     */
    template <typename Element, typename Compute>
    struct PrecArray3D : public PrecArayBase<Compute, PrecSums3D<Compute>>
    {
        typedef PrecSums3D<Compute> SumsType;                   //!< Type of precomputed sums.
        typedef PrecArayBase<Compute, SumsType> BaseType;       //!< Specialization of PrecArayBase.
        typedef Element ElementType;                            //!< Type for data element storage.
        typedef typename BaseType::ComputeType ComputeType;     //!< Type for precise computation.
        typedef typename BaseType::EigenType EigenType;         //!< Underlying Eigen type.

        //! Precompute sums for given points.
        /*!
         * Points are organized in std::vector and the function rely on fact that they are stored in correct order in continuous chunk of memory.
         * @param vec points for precomputation.
         */
        void precompute(const std::vector<rtl::Vector3D<ElementType>> &vec)
        {
            size_t vec_size = vec.size();
            BaseType::resize(vec_size);

            Eigen::Map<const Eigen::Matrix<ElementType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> map(&vec[0][0], vec_size, 3);
            BaseType::array.block(1, SumsType::cx, vec_size, 1) = map.array().col(SumsType::cx).template cast<ComputeType>();
            BaseType::array.block(1, SumsType::cy, vec_size, 1) = map.array().col(SumsType::cy).template cast<ComputeType>();
            BaseType::array.block(1, SumsType::cz, vec_size, 1) = map.array().col(SumsType::cz).template cast<ComputeType>();

            BaseType::array.block(1, SumsType::cx2, vec_size, 1) = BaseType::array.block(1, SumsType::cx, vec_size, 1) * BaseType::array.block(1, SumsType::cx, vec_size, 1);
            BaseType::array.block(1, SumsType::cy2, vec_size, 1) = BaseType::array.block(1, SumsType::cy, vec_size, 1) * BaseType::array.block(1, SumsType::cy, vec_size, 1);
            BaseType::array.block(1, SumsType::cz2, vec_size, 1) = BaseType::array.block(1, SumsType::cz, vec_size, 1) * BaseType::array.block(1, SumsType::cz, vec_size, 1);

            BaseType::array.block(1, SumsType::cxy, vec_size, 1) = BaseType::array.block(1, SumsType::cx, vec_size, 1) * BaseType::array.block(1, SumsType::cy, vec_size, 1);
            BaseType::array.block(1, SumsType::cyz, vec_size, 1) = BaseType::array.block(1, SumsType::cy, vec_size, 1) * BaseType::array.block(1, SumsType::cz, vec_size, 1);
            BaseType::array.block(1, SumsType::czx, vec_size, 1) = BaseType::array.block(1, SumsType::cz, vec_size, 1) * BaseType::array.block(1, SumsType::cx, vec_size, 1);

            for (size_t i = 1; i < vec_size + 1; i++)
                BaseType::array.row(i) += BaseType::array.row(i - 1);
        }
    };
}

#endif //ROBOTICTEMPLATELIBRARY_VECT_PRECARRAY_H
