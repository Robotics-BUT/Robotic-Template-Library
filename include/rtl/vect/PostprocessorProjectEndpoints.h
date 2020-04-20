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

#ifndef ROBOTICTEMPLATELIBRARY_VECT_POSTPROCESSORPROJECTENDPOINTS_H
#define ROBOTICTEMPLATELIBRARY_VECT_POSTPROCESSORPROJECTENDPOINTS_H

namespace rtl
{
    //! Trims approximations with their end points to produce constrained output primitives.
    /*!
     * All approximations are given by a set of points in a limited area, but are infinite in some direction (lines in one, planes in two etc.).
     * End-point projection limits infinite approximations to appropriate constrained types (line segments, polygons) covering only the area of interest.
     * @tparam Approximation type approximation used.
     */
    template <class Approximation>
    class PostprocessorProjectEndpoints
    {
    public:
        typedef typename Approximation::ElementType ElementType;        //!< Base type of stored elements.
        typedef typename Approximation::ComputeType ComputeType;        //!< Base type for precise computations.
        typedef typename Approximation::VectorType VectorType;          //!< VectorND specialization corresponding to given \p Approximation.
        typedef typename Approximation::ConstrainedType OutputType;     //!< Type of constrained line approximation.
        typedef std::pair<size_t, size_t> IndexType;                    //!< Type holding a pair of indices to an array.

        //! Default constructor.
        PostprocessorProjectEndpoints() = default;

        //! Default destructor.
        ~PostprocessorProjectEndpoints() = default;

        //! Read-only access to extracted constrained primitives.
        /*!
         *
         * @return constrained approximations.
         */
        const std::vector<OutputType>& output() const { return int_output; }

        //! Functor call for constrained primitives extraction.
        /*!
         * Constrained objects are generated using all points covered by the approximation.
         * @param pts points being processed.
         * @param approximations approximations being trimmed.
         * @param indices range indices of the approximations.
         * @return true on success, false otherwise.
         */
        bool operator()(const std::vector<VectorType> &pts, const std::vector<Approximation> &approximations, const std::vector<IndexType> &indices)
        {
            if (approximations.size() != indices.size() || approximations.size() == 0)
                return false;
            int_output.clear();
            int_output.reserve(approximations.size());
            for (size_t i = 0; i < approximations.size(); i++)
                int_output.emplace_back(approximations[i].trim(pts.begin() + indices[i].first, pts.begin() + indices[i].second));
            return true;
        }

    private:
        std::vector<OutputType> int_output;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_VECT_POSTPROCESSORPROJECTENDPOINTS_H
