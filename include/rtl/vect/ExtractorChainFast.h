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

#ifndef ROBOTICTEMPLATELIBRARY_VECT_EXTRACTORCHAINFAST_H
#define ROBOTICTEMPLATELIBRARY_VECT_EXTRACTORCHAINFAST_H

namespace rtl
{
    //! Extracts geometrical primitives from an ordered point cloud.
    /*!
     * Approximation fitting works on the binary search principle - to fit an approximation to the longest section of \a N point chain,
     * roughly \a log \a N computations is required.
     * @tparam SumArray type of the array of precomputed sums used.
     * @tparam Approximation type of approximation to be fitted to the data.
     */
    template<class SumArray, class Approximation>
    class ExtractorChainFast
    {
    public:
        typedef typename Approximation::ElementType ElementType;    //!< Base type of stored elements.
        typedef typename Approximation::ComputeType ComputeType;    //!< Base type for precise computations.
        typedef std::pair <size_t, size_t> IndexType;               //!< Type holding a pair of indices to an array.

        //! Default constructor.
        ExtractorChainFast() = default;

        //! Default destructor.
        ~ExtractorChainFast() = default;

        //! Sets maximal permitted standard deviation of point-approximation distances.
        /*!
         *
         * @param sigma new standard deviation.
         */
        void setSigma(ElementType sigma) { err2 = sigma * sigma; }

        //! Functor call for processing of an array of precomputed sums.
        /*!
         * Output parameters are always cleared before the extraction process. Order of extracted primitives corresponds to their order
         * in the precomputed sums (and therefore in the point cloud). Corresponding approximations and indices have the same index in output vectors.
         * @param sum_array precomputed sums to be processed.
         * @param approximations output parameter for found approximations.
         * @param indices output parameter for indices defining valid range for \p approximations.
         * @return true on success, false otherwise.
         */
        bool operator()(const SumArray &sum_array, std::vector <Approximation> &approximations, std::vector <IndexType> &indices)
        {
            beg_i = 0;
            last_pt = sum_array.size() - 1;
            end_i = last_pt;
            n = end_i;

            approximations.clear();
            indices.clear();

            while (true)
            {
                appr(sum_array.sums(beg_i, end_i));
                if (appr.errSquared() < err2)
                {
                    if (end_i == last_pt)
                    {
                        approximations.push_back(appr);
                        indices.emplace_back(beg_i, end_i);
                        break;
                    }
                    if (n == 0)
                    {
                        approximations.push_back(appr);
                        indices.emplace_back(beg_i, end_i);
                        beg_i = end_i;
                        end_i = last_pt;

                        if (beg_i > end_i - 2)
                            beg_i = end_i - 2;

                        n = end_i - beg_i;
                        continue;
                    }

                    if (n % 2)
                        n = n / 2 + 1;
                    else
                        n = n / 2;

                    end_i = end_i + n;
                    if (end_i > last_pt)  // against possible array overflow
                        end_i = last_pt;
                } else
                {
                    if (n == 0)
                    {
                        if (end_i - beg_i == 2)
                        {
                            if (end_i == last_pt)
                            {
                                approximations.push_back(appr);
                                indices.emplace_back(beg_i, end_i);
                                break;
                            }

                            approximations.push_back(appr);
                            indices.emplace_back(beg_i, end_i);
                            beg_i = end_i;
                            end_i = last_pt;
                            if (beg_i > end_i - 2)
                                beg_i = end_i - 2;

                            n = end_i - beg_i;
                            continue;
                        } else
                            end_i = end_i - 1;
                    } else
                    {
                        n = n / 2;
                        end_i = end_i - n;
                    }
                }
            }
            return true;
        }

    private:
        Approximation appr;
        size_t beg_i{}, end_i{}, last_pt{}, n{};
        ElementType err2;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_VECT_EXTRACTORCHAINFAST_H
