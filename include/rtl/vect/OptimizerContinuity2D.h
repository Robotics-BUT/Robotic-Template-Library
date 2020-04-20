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

#ifndef ROBOTICTEMPLATELIBRARY_VECT_OPTIMIZERCONTINUITY2D_H
#define ROBOTICTEMPLATELIBRARY_VECT_OPTIMIZERCONTINUITY2D_H

namespace rtl
{
    //! Checks intersections and fixes 2D line approximations, if continuous output polyline is required.
    /*!
     * If a complex shape of an ordered point cloud is vectorized, approximations near inflexion points of the cloud might not intersect
     * close to the cloud (in principle they might not intersect at all). If a continuous polyline is required as the vectorization output,
     * these artifacts can be fixed by addition of an extra approximation bridging the inflexion point. Requires array of precomputed sums
     * to work.
     * @tparam SumArray type of precomputed sums.
     * @tparam Approximation type approximation used.
     */
    template <class SumArray, class Approximation>
    class OptimizerContinuity2D
    {
    public:
        typedef typename Approximation::ElementType ElementType;            //!< Base type of stored elements.
        typedef typename Approximation::ComputeType ComputeType;            //!< Base type for precise computations.
        typedef typename Approximation::VectorType VectorType;              //!< VectorND specialization corresponding to given \p Approximation.
        typedef typename Approximation::ConstrainedType LineSegmentType;    //!< Type of constrained line approximation.
        typedef std::pair<size_t, size_t> IndexType;                        //!< Type holding a pair of indices to an array.

        //! Default constructor.
        OptimizerContinuity2D() = default;

        //! Default destructor.
        ~OptimizerContinuity2D() = default;

        //! Sets maximal permitted distance of intersection of the approximation lines from their neighbouring end points.
        /*!
         * \p delta should be circa 3-10 times larger that standard deviation used in preceding vectorization to avoid unnecessary splits of approximations
         * and maintain functionality of the optimization procedure.
         * @param delta new value of the maximal distance.
         */
        void setDelta(ElementType delta) { delta2 = delta * delta; }

        //! Functor call for optimization of the previously obtained approximations.
        /*!
         * During the optimization process \p lines and \p indices might change content as well as size,
         * but the corresponding approximations and indices still have the same index in these input/output vectors.
         * @param pts vectorized points.
         * @param sum_array precomputed sums.
         * @param lines linear approximations to be optimized.
         * @param indices range indices of the approximations to be optimized.
         * @return true on success, false otherwise.
         */
        bool operator()(const std::vector<VectorType> &pts, const SumArray &sum_array, std::vector<Approximation> &lines, std::vector<IndexType> &indices)
        {
            if (lines.size() < 2)
                return true;
            else
            {
                size_t i = 1;
                while (i < lines.size())
                {
                    if (!Approximation::getCrossing(lines[i - 1], lines[i], v_tmp) ||
                        VectorType::distanceSquared(v_tmp, (pts[indices[i - 1].second - 1] + pts[indices[i - 1].second]) / 2) > delta2)
                    {
                        if (indices[i].second - indices[i - 1].first < 6)
                            return false;
                        else
                        {
                            m1 = (2 * indices[i - 1].first + indices[i].second) / 3;
                            m2 = (indices[i - 1].first + 2 * indices[i].second) / 3;

                            app1(sum_array.sums(indices[i - 1].first, m1));
                            app2(sum_array.sums(m1, m2));
                            app3(sum_array.sums(m2, indices[i].second));

                            lines[i - 1] = app1;
                            indices[i - 1] = std::make_pair(indices[i - 1].first, m1);
                            lines[i] = app3;
                            indices[i] = std::make_pair(m2, indices[i].second);

                            lines.insert(lines.begin() + i, app2);
                            indices.insert(indices.begin() + i, IndexType(m1, m2));
                            continue;
                        }
                    }
                    i++;
                }
            }
            return true;
        }

    private:
        ElementType delta2;
        VectorType v_tmp;
        size_t m1{}, m2{};
        Approximation app1, app2, app3;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_VECT_OPTIMIZERCONTINUITY2D_H
