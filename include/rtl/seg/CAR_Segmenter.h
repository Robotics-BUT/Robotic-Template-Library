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

#ifndef ROBOTICTEMPLATELIBRARY_SEG_CAR_SEGMENTER_H
#define ROBOTICTEMPLATELIBRARY_SEG_CAR_SEGMENTER_H

#include <list>
#include <vector>
#include <map>

#include "rtl/Core.h"

namespace rtl
{
    //! Segmenter class for partitioning of ordered and closed point clouds into continuous clusters.
    /*!
     * The abbreviation stands for:
     * \li Cyclic - works on closed chains of points.
     * \li Adaptive - point neighbourhood scales with distance from the origin.
     * \li Retroactive - rewrites pertinence on merging clusters.
     *
     * The CAR_Segmenter loads and processes the data as a single cyclic point cloud and sorts them into continuous clusters. The criterion of continuity is the proximity test as follows:
     * \li Distance threshold is computed - it is proportional to the number of neighbour points examined given by setStepSize() and a distance of the point of interest from origin (specified in loadData() second parameter).
     * \li Distance threshold is clipped by the lower and upper bounds specified during construction or explicitly by setLowerBound() or setUpperBound().
     * \li Required number of neighbours is examined and if a point-neighbour distance is lower than the threshold, the are set to belong to the same cluster.
     * \li If one point can belong to more clusters, the clusters are merged.
     * \li If the point does not have any close enough neighbour, a new cluster is created.
     *
     * @tparam Vector base VectorND specialization.
     */
    template <class Vector>
    class CAR_Segmenter
    {
    public:
        typedef typename Vector::ElementType ElementType;   //!< Base type for vector elements.
        typedef Vector VectorType;                          //!< Base VectorND specialization.

        //! Default constructor.
        CAR_Segmenter() { cluster_counter = 1; }

        //! Parameterized constructor.
        /*!
         * Sets given parameters during construction.
         * @param step number of points tested in the proximity test.
         * @param lower_bound minimal value of the proximity threshold.
         * @param upper_bound maximal value of the proximity threshold.
         */
        CAR_Segmenter(size_t step, ElementType lower_bound, ElementType upper_bound)
        {
            (step == 0) ? step_size = 1 : step_size = step;
            l_bound2 = lower_bound * lower_bound;
            u_bound2 = upper_bound * upper_bound;
        }

        //! Default destructor.
        ~CAR_Segmenter() = default;

        //! Sets number of points tested in the proximity test.
        /*!
         *
         * @param step new number of tested points.
         */
        void setStepSize(size_t step) { step_size = step; }

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

        //! Reserves internal buffers to accept given number of input points.
        /*!
         *
         * @param max_size maximal size of the input.
         */
        void setMaxSize(size_t max_size) { cluster_pertinence.reserve(max_size); }

        //! Gives number of available clusters after the segmentation.
        /*!
         *
         * @return number of available clusters.
         */
        size_t clustersAvailable() { return clusters.size(); }

        //! Loads and processes a new point cloud.
        /*!
         * Requires an ordered point cloud and will test for circular continuity.
         * @param points the point cloud.
         * @param origin point from which the point cloud was obtained. VectorType::zero() by default.
         */
        void loadData(const std::vector<VectorType> &points, const VectorType &origin = VectorType::zeros())
        {
            if (points.empty())
                return;

            cluster_counter = 0;
            if(cluster_pertinence.capacity() >= points.size())
                cluster_pertinence.clear();
            else
                cluster_pertinence.reserve(points.size());
            clusters.clear();
            bool has_cluster;
            std::vector<size_t> first_occurence;
            ElementType dist2, scale_factor = (ElementType)step_size * 2.0f * C_PI / points.size();
            scale_factor *= scale_factor;

            // cluster pertinence search
            for (size_t i = 0; i < points.size(); i++)
            {
                has_cluster = false;
                for (size_t j = 1; j < step_size; j++)
                {
                    if (j > i)
                        break;

                    dist2 = scale_factor * VectorType::distanceSquared(points[i], origin);
                    if (dist2 < l_bound2)
                        dist2 = l_bound2;
                    if (dist2 > u_bound2)
                        dist2 = u_bound2;
                    if (VectorType::distanceSquared(points[i], points[i - j]) < dist2)
                    {
                        if (!has_cluster)
                        {
                            has_cluster = true;
                            cluster_pertinence.push_back(cluster_pertinence[i - j]);
                        }
                        else if (cluster_pertinence[i] != cluster_pertinence[i - j])
                        {
                            size_t merged_cluster = cluster_pertinence[i - j], stop = first_occurence[cluster_pertinence[i - j]];
                            for (size_t k = i - j; k >= stop && k < cluster_pertinence.size(); k--)
                            {
                                if (cluster_pertinence[k] == merged_cluster)
                                {
                                    cluster_pertinence[k] = cluster_pertinence[i];
                                    if (k < first_occurence[cluster_pertinence[i]])
                                        first_occurence[cluster_pertinence[i]] = k;
                                }
                            }
                        }
                    }
                }
                if (!has_cluster)
                {
                    cluster_pertinence.push_back(cluster_counter);
                    first_occurence.push_back(i);
                    cluster_counter++;
                }
            }

            // close the loop
            for (size_t i = 0; i < step_size; i++)
            {
                for (size_t j = points.size() - 1; j > points.size() - 1 - i; j--)
                {
                    dist2 = scale_factor * points[i].lengthSquared();
                    if (dist2 < l_bound2)
                        dist2 = l_bound2;
                    if (dist2 > u_bound2)
                        dist2 = u_bound2;
                    if (VectorType::distanceSquared(points[i], points[j]) < dist2 && cluster_pertinence[i] != cluster_pertinence[j])
                    {
                        size_t merged_cluster = cluster_pertinence[j], stop = first_occurence[cluster_pertinence[j]];
                        for (size_t k = j; k >= stop && k < cluster_pertinence.size(); k--)
                        {
                            if (cluster_pertinence[k] == merged_cluster)
                            {
                                cluster_pertinence[k] = cluster_pertinence[i];
                                first_occurence[cluster_pertinence[i]] = k;
                            }
                        }
                    }
                }
            }

            // sort the points to clusters
            std::map<size_t, std::vector<VectorType>> tmp_clusters;
            for (size_t i = 0; i < cluster_pertinence.size(); i++)
            {
                if (points[i].hasNaN()) // filtration of invalid points
                    continue;
                if (first_occurence[cluster_pertinence[i]] <= i)
                    clusters[cluster_pertinence[i]].push_back(points[i]);
                else
                    tmp_clusters[cluster_pertinence[i]].push_back(points[i]);
            }
            for (auto & tmp_cluster : tmp_clusters)
            {
                if (!tmp_cluster.second.empty()) // filtration of empty clusters
                    clusters[tmp_cluster.first].insert(clusters[tmp_cluster.first].end(), std::make_move_iterator(tmp_cluster.second.begin()), std::make_move_iterator(tmp_cluster.second.end()));
            }
        }

        //! Returns one ordered and continuous cluster of points.
        /*!
         * The points are moved in C++ sense, which makes grabbing of the cluster fast, but the points are removed from segmenter's buffer
         * and clustersAvailable() will return a value reduced by one after grabbing.
         * @return moved std::vector of points, or an empty vector, if no clusters are available.
         */
        std::vector<VectorType> grabCluster()
        {
            if (!clusters.empty())
            {
                auto it = clusters.begin();
                auto vect = std::move(it->second);
                clusters.erase(it);
                return vect;
            }
            else
                return std::vector<VectorType>();
        }

    private:
        size_t cluster_counter{}, step_size{};
        ElementType l_bound2{}, u_bound2{};
        std::vector<size_t> cluster_pertinence;
        std::map<size_t, std::vector<VectorType>> clusters;
    };
}

#endif // ROBOTICTEMPLATELIBRARY_SEG_CAR_SEGMENTER_H
