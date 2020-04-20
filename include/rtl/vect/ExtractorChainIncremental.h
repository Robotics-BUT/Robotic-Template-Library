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

#ifndef ROBOTICTEMPLATELIBRARY_VECT_EXTRACTORCHAININCREMENTAL_H
#define ROBOTICTEMPLATELIBRARY_VECT_EXTRACTORCHAININCREMENTAL_H

namespace rtl
{
    //! Extracts geometrical primitives from a continuous stream from an ordered point cloud.
    /*!
     * Gradually adds points to the approximation until the threshold given by setSigma() is passed, the approximation is saved and theprocess is repeated.
     * This method does not require an array of precomputed data, but is slow because the approximation is recomputed for each point.
     * @tparam Approximation type of approximation to be fitted to the data.
     */
    template<class Approximation>
    class ExtractorChainIncremental
    {
    public:
        typedef typename Approximation::ElementType ElementType;    //!< Base type of stored elements.
        typedef typename Approximation::ComputeType ComputeType;    //!< Base type for precise computations.
        typedef typename Approximation::VectorType VectorType;      //!< VectorND specialization corresponding to given \p Approximation.
        typedef typename Approximation::PrecSumsType PrecSumsType;  //!< Precomputed sums required by the \p Approximation.
        typedef std::pair<size_t, size_t> IndexType;                //!< Type holding a pair of indices to an array.

        //! Default constructor.
        ExtractorChainIncremental()= default;

        //! Default destructor.
        ~ExtractorChainIncremental()= default;

        //! Sets maximal permitted standard deviation of point-approximation distances.
        /*!
         *
         * @param sigma new standard deviation.
         */
        void setSigma(ElementType sigma) { err2 = sigma * sigma; }

        //! Functor call for processing an ordered point cloud.
        /*!
         * Output parameters are always cleared before the extraction process. Order of extracted primitives corresponds to their order
         * in the precomputed sums (and therefore in the point cloud). Corresponding approximations and indices have the same index in output vectors.
         * @param pts points to be processed.
         * @param approximations output parameter for found approximations.
         * @param indices output parameter for indices defining valid range for \p approximations.
         * @return true on success, false otherwise.
         */
        bool operator()(const std::vector<VectorType> &pts, std::vector<Approximation> &approximations, std::vector<IndexType> &indices)
        {
            beg_i = 0;
            end_i = 0;
            sums.setZero();

            approximations.clear();
            indices.clear();

            while (end_i < pts.size())
            {
                PrecSumsType ps(pts[end_i].template cast<ComputeType>());
                sums += ps;
                if (Approximation::getErrorSquared(sums) < err2)
                {
                    end_i++;
                    continue;
                }

                approximations.emplace_back(sums - ps);
                indices.emplace_back(beg_i, end_i);

                sums = ps;
                beg_i = end_i;
                end_i++;
            }
            if (end_i - beg_i >= 2)
            {
                approximations.emplace_back(sums);
                indices.emplace_back(beg_i, end_i);
            }
            return true;
        }

    private:
        size_t beg_i{}, end_i{};
        ElementType err2;
        PrecSumsType sums;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_VECT_EXTRACTORCHAININCREMENTAL_H
