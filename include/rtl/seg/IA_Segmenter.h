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

#ifndef ROBOTICTEMPLATELIBRARY_SEG_IA_SEGMENTER_H
#define ROBOTICTEMPLATELIBRARY_SEG_IA_SEGMENTER_H

#include <list>
#include <vector>
#include <map>

#include "rtl/Core.h"

namespace rtl
{
    //! Segmenter class for partitioning of continuous stream of points clusters.
    /*!
     * The abbreviation stands for:
     * \li Incremental - segments continuous stream of points
     * \li Adaptive - point neighbourhood scales with distance from the origin.
     *
     * The IA_Segmenter continualy accepts points through the addPoint() method and sorts them into continuous clusters. The criterion of continuity is the proximity test as follows:
     * \li Distance threshold is computed - it is proportional to the scale factor given by setScaling() and a distance of the point of interest from origin (specified in loadData() second parameter).
     * \li Distance threshold is clipped by the lower and upper bounds specified during construction or explicitly by setLowerBound() or setUpperBound().
     * \li Required number of neighbours is examined and if a point-neighbour distance is lower than the threshold, the are set to belong to the same cluster.
     * \li If one point can belong to more clusters, the clusters are merged.
     * \li If the point does not have any close enough neighbour, a new alive cluster is created.
     * \li The cluster expanded with a new point the longest time ago is checked and is its last update happened setStep() points before, it is moved from alive clusters to closed clusters and cannot be expanded any more.
     *
     * @tparam Vector base VectorND specialization.
     */
    template <class Vector>
    class IA_Segmenter
    {
    public:
        typedef typename Vector::ElementType ElementType;   //!< Base type for vector elements.
        typedef Vector VectorType;                          //!< Base VectorND specialization.

        //! Default constructor.
        IA_Segmenter() { cluster_counter = 1; }

        //! Parameterized constructor.
        /*!
         * Sets given parameters during construction.
         * @param step number of points tested in the proximity test.
         * @param lower_bound minimal value of the proximity threshold.
         * @param upper_bound maximal value of the proximity threshold.
         * @param scaling scaling of the point's neighbourhood with distance from origin.
         */
        IA_Segmenter(size_t step, ElementType lower_bound, ElementType upper_bound, ElementType scaling)
        {
            (step == 0) ? step_size = 1 : step_size = step;
            scale_factor2 = scaling * scaling;
            l_bound2 = lower_bound * lower_bound;
            u_bound2 = upper_bound * upper_bound;
        }

        //! Default destructor.
        ~IA_Segmenter() = default;

        //! Sets number of points tested in the proximity test.
        /*!
         *
         * @param step new number of tested points.
         */
        void setStepSize(unsigned int step) { step_size = step; }

        //! Sets lower bound in the proximity test.
        /*!
         *
         * @param lb new lower bound value.
         */
        void setLowerBound(ElementType lb) { l_bound2 = lb * lb; }

        //! Sets upper bound in the proximity test.
        /*!
         *
         * @param ub new upper bound value.
         */
        void setUpperBound(ElementType ub) { u_bound2 = ub * ub; }

        //! Sets scaling of the point's neigbourhood with distance.
        /*!
         *
         * @param scale new scale factor.
         */
        void setScaling(ElementType scale) { scale_factor2 = scale * scale; }

        //! Number of closed clusters available to be grabbed by grabCluster().
        /*!
         *
         * @return number of closed clusters.
         */
        size_t closedClustersAvailable() { return clusters_closed.size(); }

        //! Number of alive clusters with potential to be expanded.
        /*!
         * These clusters cannot be grabbed, only read through aliveClusters().
         * @return number of alive clusters.
         */
        size_t aliveClustersAvailable() { return clusters_alive.size(); }

        //! Returns read-only reference to alive clusters.
        /*!
         *
         * @return reference to alive clusters.
         */
        const std::map<size_t , std::vector<VectorType>>& aliveClusters() const { return  clusters_alive; }

        //! Processes a new point and adds it to appropriate cluster.
        /*!
         *
         * @param pt point to be processed.
         * @param origin point from which the added point was obtained. VectorType::zero() by default.
         */
        void addPoint(const VectorType &pt, const VectorType &origin = VectorType::zeros())
        {
            bool has_cluster = false;
            size_t cl_p = -1;

            // cluster pertinence search
            for (auto it = points.begin(); it != points.end(); it++)
            {
                ElementType  dist2 = scale_factor2 * VectorType::distanceSquared(pt, origin);
                if (dist2 < l_bound2)
                    dist2 = l_bound2;
                if (dist2 > u_bound2)
                    dist2 = u_bound2;

                if (VectorType::distanceSquared(pt, it->pt) < dist2)
                {
                    if (!has_cluster)
                    {
                        has_cluster = true;
                        cl_p = it->pert;
                    }
                    else if (cl_p != it->pert)
                    {
                        for (auto merge_it = points.begin(); merge_it != points.end(); merge_it++)
                        {
                            if (merge_it->pert == cl_p)
                            {
                                merge_it->pert = it->pert;
                            }
                        }
                        cl_p = it->pert;
                    }
                }
            }

            if (!has_cluster)
            {
                cl_p = cluster_counter++;
            }

            PointPert pp;
            pp.pt = pt;
            pp.pert = cl_p;
            points.emplace_back(pp);
            clusters_alive_refs[cl_p]++;

            if (points.size() > step_size)
            {
                auto to_cl = points.front();
                points.pop_front();
                clusters_alive[to_cl.pert].push_back(to_cl.pt);
                clusters_alive_refs[to_cl.pert]--;

                if (clusters_alive_refs[to_cl.pert] == 0)
                {
                    auto closed_cl = clusters_alive.extract(to_cl.pert);
                    clusters_closed.insert(std::move(closed_cl));
                }
            }
        }

        //! Returns one ordered and continuous cluster of points.
        /*!
         * The points are moved in C++ sense, which makes grabbing of the cluster fast, but the points are removed from segmenter's buffer
         * and closedClustersAvailable() will return a value reduced by one after grabbing.
         * @return moved std::vector of points, or an empty vector, if no clusters are available.
         */
        std::vector<VectorType> grabCluster()
        {
            if (!clusters_closed.empty())
            {
                auto it = clusters_closed.begin();
                auto vect = std::move(it->second);
                clusters_closed.erase(it);
                return vect;
            }
            else
                return std::vector<VectorType>();
        }

    private:
        struct PointPert
        {
            VectorType pt;
            size_t pert;
        };

        size_t cluster_counter{}, step_size{};
        ElementType l_bound2{}, u_bound2{}, scale_factor2{};
        std::list<PointPert> points;
        std::map<size_t , std::vector<VectorType>> clusters_closed, clusters_alive;
        std::map<size_t , size_t> clusters_alive_refs;
    };
}
#endif // ROBOTICTEMPLATELIBRARY_SEG_IA_SEGMENTER_H
