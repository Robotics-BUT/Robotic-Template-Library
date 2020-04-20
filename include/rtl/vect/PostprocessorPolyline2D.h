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

#ifndef ROBOTICTEMPLATELIBRARY_VECT_POSTPROCESSORPOLYLINE2D_H
#define ROBOTICTEMPLATELIBRARY_VECT_POSTPROCESSORPOLYLINE2D_H

namespace rtl
{
    //! Generates polyline output from linear approximation in 2D.
    /*!
     * Polyline postprocessing generates a continuous replacement of the original point cloud.
     * Due to possible artifacts near inflection points, it is advised to use OptimizerContinuity2D in most cases before the polyline is extracted by this algorithm.
     * @tparam Approximation type approximation used.
     */
    template <class Approximation>
    class PostprocessorPolyline2D
    {
    public:
        typedef typename Approximation::ElementType ElementType;            //!< Base type of stored elements.
        typedef typename Approximation::ComputeType ComputeType;            //!< Base type for precise computations.
        typedef typename Approximation::VectorType VectorType;              //!< VectorND specialization corresponding to given \p Approximation.
        typedef typename Approximation::ConstrainedType LineSegmentType;    //!< Type of constrained line approximation.
        typedef std::pair<size_t, size_t> IndexType;                        //!< Type holding a pair of indices to an array.

        //! Default constructor.
        PostprocessorPolyline2D() = default;

        //! Default destructor.
        ~PostprocessorPolyline2D() = default;

        //! Generates and returns line segments forming the polyline.
        /*!
         *
         * @return line segments of the polyline.
         */
        std::vector<LineSegmentType> lineSegments() const
        {
            std::vector<LineSegmentType> output;
            if (int_polyline.size() < 2)
                return output;
            output.reserve(int_polyline.size() - 1);
            for (size_t i = 1; i < int_polyline.size(); i++)
                output.emplace_back(int_polyline[i - 1], int_polyline[i]);
            return output;
        }

        //! Functor call for polyline extraction.
        /*!
         * Computes intersections of the approximations, projects end-points and forms an internal polyline.
         * @param pts points being processed.
         * @param lines linear approximations.
         * @param indices range indices of the approximations.
         * @return true on success, false otherwise.
         */
        bool operator()(const std::vector<VectorType> &pts, const std::vector<Approximation> &lines, const std::vector<IndexType> &indices)
        {
            if (lines.size() != indices.size() || lines.size() == 0)
                return false;
            int_polyline.clear();
            int_polyline.emplace_back(lines.front().project(pts.front()));
            for (size_t i = 1; i < lines.size(); i++)
            {
                if (!Approximation::getCrossing(lines[i - 1], lines[i], vec_tmp))
                    return false;
                int_polyline.push_back(vec_tmp);
            }
            int_polyline.emplace_back(lines.back().project(pts.back()));
            return true;
        }

    private:
        std::vector<VectorType> int_polyline;
        VectorType vec_tmp;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_VECT_POSTPROCESSORPOLYLINE2D_H
