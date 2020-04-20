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

#ifndef ROBOTICTEMPLATELIBRARY_VECT_VECTORIZERPOINTELIMINATION_H
#define ROBOTICTEMPLATELIBRARY_VECT_VECTORIZERPOINTELIMINATION_H

#include <vector>
#include <limits>

#include "rtl/Core.h"

namespace rtl
{
    //! Douglas-Peucker polyline simplification algorithm.
    /*!
     * Can be used to simplify ordered points clouds as well.
     * @tparam dimensions dimensionality of the processed point cloud.
     * @tparam Element type for data element storage.
     */
    template<int dimensions, typename Element>
    class VectorizerDouglasPeuckerND
    {
    public:
        typedef Element ElementType;                                //!< Type for data element storage.
        typedef VectorND<dimensions, Element> VectorType;           //!< VectorND specialization for internal data.
        typedef LineSegmentND<dimensions, Element> LineSegmentType; //!< LineSegmentND specialization.

        //! Default constructor.
        VectorizerDouglasPeuckerND() = default;

        //! Parameter initializing constructor.
        /*!
         *
         * @param eps maximal permitted distance of points from approximation line.
         */
        explicit VectorizerDouglasPeuckerND(ElementType eps) : epsilon2(eps * eps) {}

        //! Default destructor.
        ~VectorizerDouglasPeuckerND() = default;

        //! Sets maximal permitted distance of points from approximation line.
        /*!
         *
         * @param eps new maximal permitted distance of points from approximation line.
         */
        void setEpsilon(ElementType eps) { epsilon2 = eps * eps; }

        //! Reserves memory in internal buffers for better performance.
        /*!
         *
         * @param size expected maximal number of break points of extracted polylines.
         */
        void setMaxSize(size_t size) { break_pts.reserve(size); polyline.reserve(size + 1); }

        //! Functor call for simplification of input ordered point cloud.
        /*!
         *
         * @param input ordered point cloud.
         * @param output selected points approximating overall shape of the point cloud.
         */
        void operator()(std::vector<VectorType> & input, std::vector<LineSegmentType> & output)
        {
            break_pts.clear();
            polyline.clear();
            work_pt = 0;

            break_pts.push_back(input.size() - 1);

            while (!break_pts.empty())
            {
                if (break_pts.back() - work_pt < 2)
                {
                    polyline.push_back(input[work_pt]);
                    work_pt = break_pts.back();
                    break_pts.pop_back();
                }
                else
                {
                    max_dist = 0;
                    max_dist_index = 0;

                    LineSegmentType ls(input[work_pt], input[break_pts.back()]);

                    for (size_t i = work_pt + 1; i < break_pts.back(); i++)
                    {
                        d = ls.distanceToPointSquared(input[i]);
                        if (d > max_dist)
                        {
                            max_dist = d;
                            max_dist_index = i;
                        }
                    }

                    if (max_dist > epsilon2)
                        break_pts.push_back(max_dist_index);
                    else
                    {
                        polyline.push_back(input[work_pt]);
                        work_pt = break_pts.back();
                        break_pts.pop_back();
                    }
                }
            }
            polyline.push_back(input[work_pt]);

            output.clear();
            for (size_t i = 1; i < polyline.size(); i++)
                output.emplace_back(polyline[i - 1], polyline[i]);
        }

    private:
        ElementType d{}, max_dist{}, epsilon2{0.000001};
        size_t work_pt{}, max_dist_index{};
        std::vector<size_t > break_pts;
        std::vector<VectorType> polyline;
    };

    //! Reumann-Witkam polyline simplification algorithm.
    /*!
     * Can be used to simplify ordered points clouds as well.
     * @tparam dimensions dimensionality of the processed point cloud.
     * @tparam Element type for data element storage.
     */
    template<int dimensions, typename Element>
    class VectorizerReumannWitkamND
    {
    public:
        typedef Element ElementType;                                //!< Type for data element storage.
        typedef VectorND<dimensions, Element> VectorType;           //!< VectorND specialization for internal data.
        typedef LineSegmentND<dimensions, Element> LineSegmentType; //!< LineSegmentND specialization.

        //! Default constructor.
        VectorizerReumannWitkamND() : epsilon2() {}

        //! Parameter initializing constructor.
        /*!
         *
         * @param eps maximal permitted distance of points from approximation line.
         */
        explicit VectorizerReumannWitkamND(ElementType eps) : epsilon2(eps * eps) {}

        //! Default destructor.
        ~VectorizerReumannWitkamND() = default;

        //! Sets maximal permitted distance of points from approximation line.
        /*!
         *
         * @param eps new maximal permitted distance of points from approximation line.
         */
        void setEpsilon(float eps) { epsilon2 = eps * eps; }

        //! Reserves memory in internal buffers for better performance.
        /*!
         *
         * @param size expected maximal number of break points of extracted polylines.
         */
        void setMaxSize(size_t size) { polyline.reserve(size + 1); }

        //! Functor call for simplification of input ordered point cloud.
        /*!
         *
         * @param input ordered point cloud.
         * @param output selected points approximating overall shape of the point cloud.
         */
        void operator()(std::vector<VectorType> &input, std::vector<LineSegmentType> &output)
        {
            polyline.clear();
            polyline.push_back(input[0]);
            kp = 0;
            wp = 1;
            tp = 2;

            while (tp < input.size())
            {
                LineSegmentType ls(input[wp], input[kp]);
                d = 0;
                while (d < epsilon2 && tp < input.size())
                {
                    d = ls.distanceToPointSquared(input[tp]);
                    tp++;
                }

                if (tp == input.size())
                {
                    polyline.push_back(input.back());
                    break;
                }

                polyline.push_back(input[tp]);
                kp = wp;
                wp = tp;
                tp++;
            }

            output.clear();
            for (size_t i = 1; i < polyline.size(); i++)
                output.emplace_back(polyline[i - 1], polyline[i]);
        }

    private:
        ElementType a{}, b{}, c{}, d{}, l{}, epsilon2{0.000001};
        size_t kp{}, wp{}, tp{};
        std::vector<VectorType> polyline;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_VECT_VECTORIZERPOINTELIMINATION_H
