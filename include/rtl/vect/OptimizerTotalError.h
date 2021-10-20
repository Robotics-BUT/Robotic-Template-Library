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

#ifndef ROBOTICTEMPLATELIBRARY_VECT_OPTIMIZERTOTALERROR_H
#define ROBOTICTEMPLATELIBRARY_VECT_OPTIMIZERTOTALERROR_H

#include <vector>
#include <map>
#include <set>

namespace rtl
{
    //! Optimize total approximation error of the whole point cloud.
    /*!
     * The optimization starts with regular vectorization output and manipulates ranges of the approximations to minimize overall error.
     * Principle of the optimization is the Nelder-Mead method modified to work in discrete space of integral indices and with constraints
     * arising from limited number of input points and strict condition on "touching" intervals of approximation.
     * @tparam SumArray type of precomputed sums.
     * @tparam Approximation type approximation used.
     */
    template <class SumArray, class Approximation>
    class OptimizerTotalError
    {
    public:
        typedef typename Approximation::ElementType ElementType;            //!< Base type of stored elements.
        typedef typename Approximation::ComputeType ComputeType;            //!< Base type for precise computations.
        typedef typename Approximation::VectorType VectorType;              //!< VectorND specialization corresponding to given \p Approximation.
        typedef typename Approximation::ConstrainedType ConstrainedType;    //!< Type of constrained approximation.
        typedef std::pair<size_t, size_t> IndexType;                        //!< Type holding a pair of indices to an array.

        //! Default constructor.
        OptimizerTotalError() = default;

        //! Default destructor.
        ~OptimizerTotalError() = default;

        //! Sets initial shift of Nelder-Mead simplex vertices with respect to the optimized solution.
        /*!
         * Optimal value is usually between \a N/50 and \a N/500, where \a N is the number of processed points. Must be at least one, which is enforced by the function itself.
         * @param simplex_shift new initial shift.
         */
        void setSimplexShift(size_t simplex_shift) { (simplex_shift > 0) ? shift = simplex_shift : shift = 1; }

        //! Sets maximal number of iterations of the method.
        /*!
         * Usually the optimization terminates when the simplex collapses due to discrete space of array indices, but for the same reason, infinite loops of simplex modifications
         * can rarely occur. This limit prevents endless loops in the program.
         * @param max_iterations new maximal number of iterations.
         */
        void setMaxIterations(size_t max_iterations) { max_iter = max_iterations; }

        //! Functor call for optimization of given approximations.
        /*!
         * During the optimization process \p approximations and \p indices might change content but not the size.
         * @param sum_array precomputed sums.
         * @param approximations approximations to be optimized.
         * @param indices range indices of the approximations to be optimized.
         * @return true on success, false otherwise.
         */
        bool operator()(const std::vector<VectorType> &, const SumArray &sum_array, std::vector<Approximation> &approximations, std::vector<IndexType> &indices)
        {
            size_t bp_cnt = approximations.size();
            if (bp_cnt < 2)
                return true;

            bp_cnt--;

            std::multimap<ElementType, size_t*> opt_vec_order;
            size_t opt_vec_size = bp_cnt * (bp_cnt + 1 + 3);
            auto bp_array = new size_t[opt_vec_size];
            size_t *bp_tmp = bp_array + bp_cnt * (bp_cnt + 1);
            size_t *bp_sums = bp_array + bp_cnt * (bp_cnt + 2);
            size_t *bp_mean = bp_array + bp_cnt * (bp_cnt + 3);
            size_t *discard, *bp_first;
            size_t i, tmp_pt, bp_last_i = sum_array.size() - 1;
            ElementType opt_err_tmp1, opt_err_tmp2;

            auto totalError = [&sum_array, bp_cnt, bp_last_i](size_t *bp)
            {
                ElementType sigma2 = 0;
                size_t sum_beg = 0;
                for (size_t i = 0; i < bp_cnt; i++)
                {
                    sigma2 += Approximation::getErrorSquared(sum_array.sums(sum_beg, bp[i]));
                    sum_beg = bp[i];
                }
                sigma2 += Approximation::getErrorSquared(sum_array.sums(sum_beg, bp_last_i));
                return sigma2;
            };

            auto forwardHomogenize = [bp_last_i, bp_cnt](size_t *bp)
            {
                if (bp[0] < 2 || bp[0] > bp_last_i)
                    bp[0] = 2;
                for (size_t i = 1; i < bp_cnt; i++)
                    if (bp[i] < bp[i - 1] + 2 || bp[i] > bp_last_i)
                        bp[i] = bp[i - 1] + 2;
            };

            auto wholeSimplexValidation = [bp_cnt, bp_array]()
            {
                size_t *array_end = bp_array + bp_cnt * (bp_cnt + 1);
                for (size_t *p_checked = bp_array + bp_cnt; p_checked < array_end; p_checked += bp_cnt)
                {
                    for (size_t *p_against = bp_array; p_against < p_checked; p_against += bp_cnt)
                    {
                        for (size_t i = 0; i < bp_cnt; i++)
                            if (p_checked[i] != p_against[i])
                                goto vertices_different;
                        return false;
                        vertices_different:;
                    }
                }
                return true;
            };

            auto newVertexValidation = [bp_cnt, &opt_vec_order](const size_t *bp)
            {
                for (auto b : opt_vec_order)
                {
                    for (size_t i = 0; i < bp_cnt; i++)
                        if (bp[i] != b.second[i])
                            goto vertices_different;
                    return false;
                    vertices_different:;
                }
                return true;
            };

            // x0 vector for Nelder-Mead search
            for (i = 0; i < bp_cnt; i++)
            {
                bp_array[i] = bp_sums[i] = indices[i].second;
                //std::cout << bp_array[i] << "\t";
            }
            //std::cout<<std::endl;

            // x1 to xn+1 vectors (the rest of the simplex)
            i = 0;
            for (size_t *pi = bp_array + bp_cnt; pi < bp_tmp; pi += bp_cnt)
            {
                for (size_t j = 0; j < bp_cnt; j++)
                {
                    if (i == j)
                        pi[j] = bp_array[j] - shift;
                    else
                        pi[j] = bp_array[j];
                    //std::cout << pi[j] << "\t";
                    bp_sums[j] += pi[j]; // sum of all of the vectors for the mean value computation
                }
                forwardHomogenize(pi);
                //std::cout<<std::endl;
                i++;
            }

            // vector order list generation
            for (size_t *pi = bp_array; pi < bp_tmp; pi += bp_cnt)
                opt_vec_order.emplace(totalError(pi), pi);

            if(!wholeSimplexValidation())
                return true;

            //int iteration_counter = 0;
            // the Nelder-Mead minimization
            for (size_t iter_cnt = 0; iter_cnt < max_iter; iter_cnt++)
            {
                //iteration_counter++;
                /*std::cout<<std::endl;
                for (auto it : opt_vec_order)
                {
                    std::cout<<it.first<<"\t";
                    for (i = 0; i < bp_cnt; i++)
                        std::cout<<it.second[i]<<"\t";
                    std::cout<<std::endl;
                }
                std::cout<<std::endl;*/
                discard = opt_vec_order.rbegin()->second;
                for (i = bp_cnt - 1; i < bp_cnt; i--)
                {
                    bp_sums[i] -= discard[i]; // remove the discarded vector from the sum m
                    bp_mean[i] = bp_sums[i] / bp_cnt; // compute the mean vector
                    tmp_pt = 2 * bp_mean[i] - discard[i]; // compute REFLECT vector
                    if (i == bp_cnt - 1)
                        (tmp_pt < bp_last_i - 1) ? bp_tmp[i] = tmp_pt : bp_tmp[i] = bp_last_i - 2;
                    else
                        (tmp_pt < bp_tmp[i + 1] - 1) ? bp_tmp[i] = tmp_pt : bp_tmp[i] = bp_tmp[i + 1] - 2;
                }
                forwardHomogenize(bp_tmp);
                opt_err_tmp1 = totalError(bp_tmp);
                if (opt_err_tmp1 < opt_vec_order.begin()->first)
                {
                    for (i = bp_cnt - 1; i < bp_cnt; i--)
                    {
                        tmp_pt = bp_tmp[i] + bp_mean[i] - discard[i]; // compute EXPAND vector
                        if (i == bp_cnt - 1)
                            (tmp_pt < bp_last_i - 1) ? discard[i] = tmp_pt : discard[i] = bp_last_i - 2;
                        else
                            (tmp_pt < discard[i + 1] - 1) ? discard[i] = tmp_pt : discard[i] = discard[i + 1] - 2;
                    }
                    forwardHomogenize(discard);
                    opt_err_tmp2 = totalError(discard);
                    if (opt_err_tmp2 < opt_err_tmp1)
                    {
                        opt_vec_order.erase(--opt_vec_order.end());
                        if(!newVertexValidation(discard))
                            break;
                        opt_vec_order.emplace(opt_err_tmp2, discard); // simplex EXPANDed
                        for (i = 0; i < bp_cnt; i++)
                            bp_sums[i] += discard[i];
                        //std::cout<<"simplex EXPANDed"<<std::endl;
                        continue;
                    }
                }
                if (opt_err_tmp1 < (++opt_vec_order.rbegin())->first)
                {
                    opt_vec_order.erase(--opt_vec_order.end());
                    if(!newVertexValidation(bp_tmp))
                        break;
                    opt_vec_order.emplace(opt_err_tmp1, bp_tmp); // simplex RFLECTed
                    for (i = 0; i < bp_cnt; i++)
                        bp_sums[i] += bp_tmp[i];
                    bp_tmp = discard;
                    //std::cout<<"simplex RFLECTed"<<std::endl;
                    continue;
                }
                if (opt_err_tmp1 < opt_vec_order.rbegin()->first)
                {
                    for (i = bp_cnt - 1; i < bp_cnt; i--)
                    {
                        tmp_pt = bp_mean[i] + (bp_mean[i] - discard[i]) / 2; // compute CONTRACT OUTSIDE vector
                        if (i == bp_cnt - 1)
                            (tmp_pt < bp_last_i - 1) ? bp_tmp[i] = tmp_pt : bp_tmp[i] = bp_last_i - 2;
                        else
                            (tmp_pt < bp_tmp[i + 1] - 1) ? bp_tmp[i] = tmp_pt : bp_tmp[i] = bp_tmp[i + 1] - 2;
                    }
                    forwardHomogenize(bp_tmp);
                    opt_err_tmp2 = totalError(bp_tmp);
                    if (opt_err_tmp2 < opt_err_tmp1)
                    {
                        opt_vec_order.erase(--opt_vec_order.end());
                        if(!newVertexValidation(bp_tmp))
                            break;
                        opt_vec_order.emplace(opt_err_tmp2, bp_tmp); // simplex CONTRACTed OUTSIDE
                        for (i = 0; i < bp_cnt; i++)
                            bp_sums[i] += bp_tmp[i];
                        bp_tmp = discard;
                        //std::cout<<"simplex CONTRACTed OUTSIDE"<<std::endl;
                        continue;
                    }
                }
                else
                {
                    for (i = bp_cnt - 1; i < bp_cnt; i--)
                    {
                        tmp_pt = bp_mean[i] + (discard[i] - bp_mean[i]) / 2; // compute CONTRACT INSIDE vector
                        if (i == bp_cnt - 1)
                            (tmp_pt < bp_last_i - 1) ? bp_tmp[i] = tmp_pt : bp_tmp[i] = bp_last_i - 2;
                        else
                            (tmp_pt < bp_tmp[i + 1] - 1) ? bp_tmp[i] = tmp_pt : bp_tmp[i] = bp_tmp[i + 1] - 2;
                    }
                    forwardHomogenize(bp_tmp);
                    opt_err_tmp2 = totalError(bp_tmp);
                    if (opt_err_tmp2 < opt_vec_order.rbegin()->first)
                    {
                        opt_vec_order.erase(--opt_vec_order.end());
                        if(!newVertexValidation(bp_tmp))
                            break;
                        opt_vec_order.emplace(opt_err_tmp2, bp_tmp); // simplex CONTRACTed INSIDE
                        for (i = 0; i < bp_cnt; i++)
                            bp_sums[i] += bp_tmp[i];
                        bp_tmp = discard;
                        //std::cout<<"simplex CONTRACTed INSIDE"<<std::endl;
                        continue;
                    }
                }
                // simplex SHRINKed
                //std::cout<<"simplex SHRINKed"<<std::endl;
                // recompute all but the first vector
                opt_err_tmp1 = opt_vec_order.begin()->first;
                bp_first = opt_vec_order.begin()->second;
                for (i = 0; i < bp_cnt; i++)
                    bp_sums[i] = bp_first[i];
                for (auto it = ++opt_vec_order.begin(); it != opt_vec_order.end(); it++)
                {
                    for (i = bp_cnt - 1; i < bp_cnt; i--)
                    {
                        tmp_pt = bp_first[i] + (it->second[i] - bp_first[i]) / 2;
                        if (i == bp_cnt - 1)
                            (tmp_pt < bp_last_i - 1) ? it->second[i] = tmp_pt : it->second[i] = bp_last_i - 2;
                        else
                            (tmp_pt < it->second[i + 1] - 1) ? it->second[i] = tmp_pt : it->second[i] = it->second[i + 1] - 2;
                        //bp_sums[i] += it->second[i];
                    }
                    forwardHomogenize(it->second);
                    for (i = 0; i < bp_cnt; i++)
                        bp_sums[i] += it->second[i];
                }
                // clear and repopulate the map with new simplices
                opt_vec_order.clear();
                opt_vec_order.emplace(opt_err_tmp1, bp_first);
                for (size_t *pi = bp_array; pi < bp_sums; pi += bp_cnt)
                {
                    if (pi != bp_first && pi != bp_tmp)
                    {
                        opt_vec_order.emplace(totalError(pi), pi);
                    }
                }
                if (!wholeSimplexValidation())
                    break;
            }

            size_t sum_beg = 0;
            bp_first = opt_vec_order.begin()->second;
            for (i = 0; i < bp_cnt; i++)
            {
                approximations[i](sum_array.sums(sum_beg, bp_first[i]));
                indices[i].first = sum_beg;
                indices[i].second = bp_first[i];
                sum_beg = bp_first[i];
            }
            approximations[i](sum_array.sums(sum_beg, bp_last_i));
            indices[i].first = sum_beg;
            indices[i].second = bp_last_i;
            return true;
        }

    private:
        size_t shift{1}, max_iter{10000};
    };
}

#endif //ROBOTICTEMPLATELIBRARY_VECT_OPTIMIZERTOTALERROR_H
