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

#ifndef ROBOTICTEMPLATELIBRARY_LINESEGMENTND_H
#define ROBOTICTEMPLATELIBRARY_LINESEGMENTND_H

#include <limits>
#include "rtl/core/VectorND.h"

namespace rtl
{
    template<int, typename>
    class TranslationND;

    template<int, typename>
    class RotationND;

    template<int, typename>
    class RigidTfND;

    //! Base template for N-dimensional line segments.
    /*!
     * Non-constructable base for LineSegmentND<> templates. Methods applicable to all dimensions are implemented in this template, while dimension specific stuff is
     * in specializations of LineSegmentND<>. This solution uses static polymorphism and does not require any extra overhead of virtual functions and runtime type
     * identification. The following notation is used in the function descriptions: \a B is the begin point, \a E is the and point and \a D is the direction vector.
     * @tparam dimensions dimensionality of the line segment.
     * @tparam Element base type of line segment coordinates.
     * @tparam ChildTemplate template inheriting this one (CRTP).
     */
    template<int dimensions, typename Element, template<int, typename> class ChildTemplate>
    class LineSegmentND_common
    {
        static_assert(dimensions > 0, "LineSegmentND must have at least one dimension");
    private:
        typedef ChildTemplate<dimensions, Element> ChildType;

    public:
        typedef Element ElementType;                        //!< Base type for line segment coordinates.
        typedef VectorND <dimensions, Element> VectorType;  //!< VectorND specialization for internal data.
        struct DistanceType                                 //!< Return type of the distance function.
        {
            DistanceType(Element d_ang, Element d_lin) : dist_lin{d_lin}, dist_ang{d_ang} {}
            Element dist_lin{0}, dist_ang{0};
            Element combined(Element c_ang, Element c_lin) { return dist_ang * c_ang + dist_lin * c_lin; }
        };

        //! Default destructor.
        ~LineSegmentND_common() = default;

        //! Returns translated copy of the line segment.
        /*!
         * @param tr the translation to be applied.
         * @return new line segment after translation.
         */
        ChildType transformed(const TranslationND<dimensions, Element> &tr) const
        {
            return ChildType(tr(int_beg), tr(int_end));
        }

        //! Translates *this line segment in-place.
        /*!
         *
         * @param tr the translation to be applied.
         */
        void transform(const TranslationND<dimensions, Element> &tr)
        {
            int_beg.transform(tr);
            int_end.transform(tr);
        }

        //! Returns rotated copy of the line segment.
        /*!
         * @param rot the rotation to be applied.
         * @return new line segment after rotation.
         */
        ChildType transformed(const RotationND<dimensions, Element> &rot) const
        {
            return ChildType(rot(int_beg), rot(int_end));
        }

        //! Rotates *this line segment in-place.
        /*!
         *
         * @param rot the rotation to be applied.
         */
        void transform(const RotationND<dimensions, Element> &rot)
        {
            int_beg.transform(rot);
            int_end.transform(rot);
            int_dir.transform(rot);
        }

        //! Returns transformed copy of the line segment.
        /*!
         * @param tf the transformation to be applied.
         * @return new line segment after transformation.
         */
        ChildType transformed(const RigidTfND<dimensions, Element> &tf) const
        {
            return ChildType(int_beg.transformed(tf), int_end.transformed(tf));
        }

        //! Transforms *this line segment in-place.
        /*!
         *
         * @param tf the transformation to be applied.
         */
        void transform(const RigidTfND<dimensions, Element> &tf)
        {
            int_beg.transform(tf);
            int_end.transform(tf);
            int_dir.transform(tf.rot());
        }

        //! Returns begin-point of the line segment.
        /*!
         *
         * @return begin-point of the line segment.
         */
        VectorType beg() const { return int_beg; }

        //! Returns end-point of the line segment.
        /*!
         *
         * @return end-point of the line segment.
         */
        VectorType end() const { return int_end; }

        //! Returns unit direction vector of the line segment.
        /*!
         *
         * @return unit direction vector of the line segment.
         */
        VectorType direction() const { return int_dir; }

        //! Returns the shortest Euclidean distance to origin.
        /*!
         *
         * @return the shortest Euclidean distance to origin.
         */
        typename VectorType::DistanceType distanceToOrigin() const { return distanceToPoint(VectorType::zeros()); }

        //! Returns the shortest Euclidean distance to given point.
        /*!
         * Requires std::sqrt(), use distanceToPointSquared() where applicable and speed is important (e.g. comparison of many distances against threshold).
         * @param point point of interest.
         * @return the shortest Euclidean distance to \p point.
         */
        typename VectorType::DistanceType distanceToPoint(const VectorType &point) const
        {
            VectorType dif = int_beg - point;
            return (dif - (dif.dot(int_dir) * int_dir)).length();
        }

        //! Returns the shortest squared Euclidean distance to given point.
        /*!
         * @param point point of interest.
         * @return the shortest squared Euclidean distance to \p point.
         */
        typename VectorType::DistanceType distanceToPointSquared(const VectorType &point) const
        {
            VectorType dif = int_beg - point;
            return (dif - (dif.dot(int_dir) * int_dir)).lengthSquared();
        }

        //! Length of the line segment.
        /*!
         * Fast, does not require square root in computation.
         * @return length of the line segment.
         */
        typename VectorType::DistanceType length() const
        {
            return VectorType::scalarProjectionOnUnit(int_end - int_beg, int_dir);
        }

        //! Set begin point.
        /*!
         * Internal direction vector is updated accordingly.
         * @param beg new location of the begin point.
         */
        void setBegin(const VectorType &beg)
        {
            int_beg = beg;
            int_dir = (int_end - int_beg).normalized();
        }

        //! Move begin point along the line segment by multiple of it's length.
        /*!
         * New begin point corresponds to \a B + \p t (\a E - \a B).
         * @param t the multiplier.
         */
        void moveBegin(Element t) { int_beg = int_beg + t * (int_end - int_beg); }

        //! Set end point.
        /*!
         * Internal direction vector is updated accordingly.
         * @param end new location of the end point.
         */
        void setEnd(const VectorType &end)
        {
            int_end = end;
            int_dir = (int_end - int_beg).normalized();
        }

        //! Move end point along the line segment by multiple of it's length.
        /*!
         * New end point corresponds to \a B + \p t (\a E - \a B).
         * @param t the multiplier.
         */
        void moveEnd(Element t) { int_end = int_beg + t * (int_end - int_beg); }

        //! Swap endpoints of the line segment and reverse its direction.
        void swapEndpoints()
        {
            VectorType tmp = int_beg;
            int_beg = int_end;
            int_end = tmp;
            int_dir = -int_dir;
        }

        //! Scalar projection of a point on the line segment using \a E - \a B vector.
        /*!
         * Finds t such that \p point = \a B + \p t (\a E - \a B).
         * @param point point of interest.
         * @return t quotient.
         */
        typename VectorType::DistanceType scalarProjection(const VectorType &point) const
        {
            return VectorType::scalarProjection(point - int_beg, int_end - int_beg);
        }

        //! Scalar projection of a point on the line segment using \a D vector.
        /*!
         * Finds t such that \p point = \a B + \p t \a D.
         * @param point point of interest.
         * @return t quotient.
         */
        typename VectorType::DistanceType scalarProjectionUnit(const VectorType &point) const
        {
            return VectorType::scalarProjectionOnUnit(point - int_beg, int_dir);
        }

        //! Projection of a point on the line segment.
        /*!
         *
         * @param point point of interest.
         * @return projection of \p point on the line segment.
         */
        VectorType vectorProjection(const VectorType &point) const
        {
            return int_beg + scalarProjectionUnit(point) * int_dir;
        }

        //! Finds the closest point to another line segment.
        /*!
         * Finds \p t such that the closest point to \p ls is \a B + \p t (\a E - \a B). If the line segments are collinear, there is no unique \p t to be computed
         * so \p t is not modified and the function returns false.
         * @param ls another line segment.
         * @param t return parameter.
         * @return true if unique \p t exists, false otherwise.
         */
        bool closestPoint(const LineSegmentND_common<dimensions, Element, ChildTemplate> &ls, typename VectorType::DistanceType &t) const
        {
            VectorType d1 = int_end - int_beg;
            VectorType d2 = ls.int_end - ls.int_beg;

            typename VectorType::ElementType d1d1 = d1.dot(d1);
            typename VectorType::ElementType d1d2 = d1.dot(d2);
            typename VectorType::ElementType d2d2 = d2.dot(d2);
            typename VectorType::ElementType det = d1d1 * d2d2 - d1d2 * d1d2;

            if (det == 0.0f)
                return false;

            VectorType b2_b1 = ls.int_beg - int_beg;
            t = (b2_b1.dot(d1) * d2d2 - b2_b1.dot(d2) * d1d2) / det;
            return true;
        }

        //! Finds the closest point to another line segment.
        /*!
         * Finds \p t such that the closest point to \p ls is \a B + \p t \a D. If the line segments are collinear, there is no unique \p t to be computed
         * so \p t is not modified and the function returns false.
         * @param ls another line segment.
         * @param t return parameter.
         * @return true if unique \p t exists, false otherwise.
         */
        bool closestPointUnit(const LineSegmentND_common<dimensions, Element, ChildTemplate> &ls, typename VectorType::DistanceType &t) const
        {
            typename VectorType::DistanceType d1d2 = VectorType::dotProduct(int_dir, ls.int_dir);
            if (d1d2 >= 1.0f)
                return false;

            typename VectorType::DistanceType det = 1 - d1d2 * d1d2;
            VectorType b2_b1 = ls.int_beg - int_beg;
            t = (b2_b1.dot(int_dir) - b2_b1.dot(ls.int_dir) * d1d2) / det;
            return true;
        }

        //! Finds crossing of the line segment with a hyperrectangle.
        /*!
         * This is going to be changed to crop by BoundingBoxND.
         *
         * Finds scalar parameters t such thad crossings with the hyperrectabgle correspond to \a B + \p t \a D.
         *
         * @param corner1 specifies one vertex of the hyperrectangle.
         * @param corner2 specifies the second vertex of the hyperrectangle.
         * @param segment_beg_t paramater corresponding to moved begin.
         * @param segment_end_t paramater corresponding to moved end.
         * @return true if the intersections exist, false otherwise.
         */
        /* Calculation:
         * vectors are denoted in capital, real numbers are small
         * dot product is denoted using dot '.'
         * n-dimensional line: B + lD = X, B = beginning, l = real parameter, D = unit direction, X = point on line
         * n-dimensional hyperplane: N.X = c, N = normal unit vector of the hyperplane, X = point on it, c = distance from origin
         * normal of the hyperplane always points into the examined area, therefore cosine of the angle between N and D demarcates if the line goes into te area (cos > 0) or leaves it (cos < 0)
         * hyperplanes demarcating te examined area are always orthogonal to remaining axes, which means that dot product is reduced to single multiplication
         * for each line/hyperplane v_tmp, the l parameter is identified
         * the line crosses the examined area if and only if all l for line entrance are smaller than all l for line exit
         */
        bool cropByHyperRect(const VectorType &corner1, const VectorType &corner2, Element &segment_beg_t, Element &segment_end_t) const
        {
            Element in_max = std::numeric_limits<Element>::lowest(), out_min = std::numeric_limits<Element>::max();
            Element l1, l2;

            for (size_t i = 0; i < dimensions; i++)
            {
                if (int_dir[i] != 0)
                {
                    if (corner1[i] < corner2[i])
                    {
                        l1 = (corner1[i] - int_beg[i]) / int_dir[i];
                        l2 = (corner2[i] - int_beg[i]) / int_dir[i];
                        if (int_dir[i] > 0)
                        {
                            if (l1 > in_max) in_max = l1;
                            if (l2 < out_min) out_min = l2;
                        } else
                        {
                            if (l1 < out_min) out_min = l1;
                            if (l2 > in_max) in_max = l2;
                        }
                    } else
                    {
                        l1 = (corner1[i] - int_beg[i]) / int_dir[i];
                        l2 = (corner2[i] - int_beg[i]) / int_dir[i];
                        if (int_dir[i] > 0)
                        {
                            if (l1 < out_min) out_min = l1;
                            if (l2 > in_max) in_max = l2;
                        } else
                        {
                            if (l1 > in_max) in_max = l1;
                            if (l2 < out_min) out_min = l2;
                        }
                    }
                }
            }

            if (in_max > out_min)
                return false;
            else
            {
                segment_beg_t = in_max;
                segment_end_t = out_min;
                return true;
            }
        }

        //! Finds crossing of the line segment with a hyperrectangle.
        /*!
         * This is going to be changed to crop by BoundingBoxND.
         *
         * Finds points on the hyperrectangle where it intersects the line segment.
         *
         * @param corner1 specifies one vertex of the hyperrectangle.
         * @param corner2 specifies the second vertex of the hyperrectangle.
         * @param segment_beg crossing on the begin side.
         * @param segment_end crossing on the end side.
         * @return true if the intersections exist, false otherwise.
         */
        bool cropByHyperRect(const VectorType &corner1, const VectorType &corner2, VectorType &segment_beg, VectorType &segment_end) const
        {
            Element l_beg, l_end;
            bool ret = cropByHyperRect(corner1, corner2, l_beg, l_end);
            if (ret)
            {
                segment_beg = int_beg + int_dir * l_beg;
                segment_end = int_beg + int_dir * l_end;
            }
            return ret;
        }

        //! Adjusts the line segment to fit into hyperrectangle.
        /*!
         * If the line segment intersects the hyperrectangle, \a B and \a E are moved to fit it exactly.
         * @param corner1 specifies one vertex of the hyperrectangle.
         * @param corner2 specifies the second vertex of the hyperrectangle.
         * @return true if the line segment intersects the hyperrectangle, false otherwise.
         */
        bool fitToHyperRect(const VectorType &corner1, const VectorType &corner2)
        {
            Element l_beg, l_end;
            bool ret = cropByHyperRect(corner1, corner2, l_beg, l_end);
            if (ret)
            {
                int_end = int_beg + int_dir * l_end;
                int_beg = int_beg + int_dir * l_beg;
            }
            return ret;
        }

        //! Finds the closest point to the other line segment for both of them.
        /*!
         * Finds \p t1 and \p t2 for \p ls1 and \p ls2 such that the closest point to the other line segment is \a B + \p t (\a E - \a B).
         * If the line segments are collinear, there are no unique \p t1 \p or t2 to be computed so they are not modified and the function returns false.
         * @param ls1 first line segment.
         * @param ls2 second line segment.
         * @param t1 parameter of \p ls1 specifying the closest point to to \p ls2.
         * @param t2 parameter of \p ls2 specifying the closest point to to \p ls1.
         * @return true if unique closest points exist, false otherwise.
         */
        static bool closestPoint(const LineSegmentND_common<dimensions, Element, ChildTemplate> &ls1, const LineSegmentND_common<dimensions, Element, ChildTemplate> &ls2, typename VectorType::DistanceType &t1, typename VectorType::DistanceType &t2)
        {
            VectorType d1 = ls1.int_end - ls1.int_beg;
            VectorType d2 = ls2.int_end - ls2.int_beg;

            typename VectorType::ElementType d1d1 = d1.dot(d1);
            typename VectorType::ElementType d2d2 = d2.dot(d2);
            typename VectorType::ElementType d1d2 = d1.dot(d2);
            typename VectorType::ElementType det = d1d1 * d2d2 - d1d2 * d1d2;

            if (det == 0)
                return false;

            VectorType b2_b1 = ls2.int_beg - ls1.int_beg;
            t1 = (b2_b1.dot(d1) * d2d2 - b2_b1.dot(d2) * d1d2) / det;
            t2 = (b2_b1.dot(d1) * d1d2 - b2_b1.dot(d2) * d1d1) / det;
            return true;
        }

        //! Finds the closest point to the other line segment for both of them.
        /*!
         * Finds \p t1 and \p t2 for \p ls1 and \p ls2 such that the closest point to the other line segment is \a B + \p t \a D.
         * If the line segments are collinear, there are no unique \p t1 \p or t2 to be computed so they are not modified and the function returns false.
         * @param ls1 first line segment.
         * @param ls2 second line segment.
         * @param t1 parameter of \p ls1 specifying the closest point to to \p ls2.
         * @param t2 parameter of \p ls2 specifying the closest point to to \p ls1.
         * @return true if unique closest points exist, false otherwise.
         */
        static bool closestPointUnit(const LineSegmentND_common<dimensions, Element, ChildTemplate> &ls1, const LineSegmentND_common<dimensions, Element, ChildTemplate> &ls2, typename VectorType::DistanceType &t1, typename VectorType::DistanceType &t2)
        {
            typename VectorType::ElementType d1d2 = ls1.int_dir.dot(ls2.int_dir);

            if (d1d2 >= 1)
                return false;

            typename VectorType::ElementType det = 1 - d1d2 * d1d2;
            VectorType b2_b1 = ls2.int_beg - ls1.int_beg;
            t1 = (b2_b1.dot(ls1.int_dir) - b2_b1.dot(ls2.int_dir) * d1d2) / det;
            t2 = (b2_b1.dot(ls1.int_dir) * d1d2 - b2_b1.dot(ls2.int_dir)) / det;
            return true;
        }

        //! Dimensionality of the vector.
        static constexpr int dimensionality() { return dimensions; }

        static DistanceType distance(const LineSegmentND_common<dimensions, Element, ChildTemplate> &l1, const LineSegmentND_common<dimensions, Element, ChildTemplate> &l2)
        {
            return DistanceType(VectorType::angleShortest(l1.direction(), l2.direction()), l2.distanceToOrigin() - l1.distanceToOrigin());
        }

        //! Return a new Line segment with all end-points vectors initialized by the user-supplied random generator.
        /*!
         * Randomness is fully specified by the callable object \p el_rnd_gen, in fact it does not have to be random at all.
         * @tparam ElRndSrc type of the random generating object.
         * @param el_rnd_gen callable object returning ElementType on invocation.
         * @return a new line segment initialized by \p el_rnd_gen.
         */
        template<class ElRndSrc>
        static ChildType random(const ElRndSrc &el_rnd_gen)
        {
            return ChildType(VectorType::random(el_rnd_gen), VectorType::random(el_rnd_gen));
        }

    protected:
        VectorType int_beg, int_end, int_dir;

        LineSegmentND_common() = default;

        LineSegmentND_common(const VectorType &beg, const VectorType &end, const VectorType dir) : int_beg{beg}, int_end{end}, int_dir{dir} {}

        LineSegmentND_common(const LineSegmentND_common &ls) : int_beg{ls.int_beg}, int_end{ls.int_end}, int_dir{ls.int_dir} {}

        explicit LineSegmentND_common(const ChildType &ls) : int_beg{ls.int_beg}, int_end{ls.int_end}, int_dir{ls.int_dir} {}

        LineSegmentND_common(const VectorType &beg, const VectorType &end) : int_beg{beg}, int_end{end}, int_dir{(end - beg).normalized()} {}

        operator ChildType() const
        {
            return ChildType(int_beg, int_end, int_dir);
        }

    };

    //! N-dimensional line segment template.
    /*!
     * General implementation of line segments of any dimension. For dimensions of special interest (2D and 3D) there are specializations with additional functionality.
     * @tparam dimensions dimensionality of the line segment.
     * @tparam Element type of the data elements.
     */
    template<int dimensions, typename Element>
    class LineSegmentND : public LineSegmentND_common<dimensions, Element, LineSegmentND>
    {
    public:
        typedef typename LineSegmentND_common<dimensions, Element, LineSegmentND>::VectorType VectorType; //!< VectorND specialization for internal data.

        //! Default constructor. No initialization is performed.
        LineSegmentND() = default;

        //! Copy constructor.
        LineSegmentND(const LineSegmentND &ls) : LineSegmentND_common<dimensions, Element, LineSegmentND>(ls) {}

        //! Construction from two end points.
        /*!
         *
         * @param beg begin point.
         * @param end end point.
         */
        LineSegmentND(const VectorType &beg, const VectorType &end) : LineSegmentND_common<dimensions, Element, LineSegmentND>(beg, end) {}

        //! Construction from both end points and a corresponding direction vector.
        /*!
         * Care must be taken to to provide a true direction vector (\a E - \a B).normalized(). No validity checks are performed and member function may lead to unexpected results with wrong \a D.
         * This constructor is meant to reduce overhead in specific cases, where \a D arise as a side effect of other computation. For regular sitation use the end-point-only constructor.
         * @param beg begin point.
         * @param end end point.
         * @param dir unit direction vector.
         */
        LineSegmentND(const VectorType &beg, const VectorType &end, const VectorType dir) : LineSegmentND_common<dimensions, Element, LineSegmentND>(beg, end, dir) {}
    };

    //! 2-dimensional line segment template specialization.
    /*!
     * 2D specialization offers some unique functions expected from classic geometry in plane, which cannot be generalized to arbitrary dimension.
     * @tparam Element type of the data elements.
     */
    template<typename Element>
    class LineSegmentND<2, Element> : public LineSegmentND_common<2, Element, LineSegmentND>
    {
    public:
        typedef typename LineSegmentND_common<2, Element, LineSegmentND>::ElementType ElementType;  //!< Base type for line segment coordinates.
        typedef typename LineSegmentND_common<2, Element, LineSegmentND>::VectorType VectorType;    //!< VectorND specialization for internal data.

        //! Default constructor. No initialization is performed.
        LineSegmentND() = default;

        //! Copy constructor.
        LineSegmentND(const LineSegmentND &ls) : LineSegmentND_common<2, Element, LineSegmentND>(ls) {}

        //! Construction from two end points.
        /*!
         *
         * @param beg begin point.
         * @param end end point.
         */
        LineSegmentND(const VectorType &beg, const VectorType &end) : LineSegmentND_common<2, Element, LineSegmentND>(beg, end) {}

        //! Construction from both end points and a corresponding direction vector.
        /*!
         * Care must be taken to to provide a true direction vector (\a E - \a B).normalized(). No validity checks are performed and member function may lead to unexpected results with wrong \a D.
         * This constructor is meant to reduce overhead in specific cases, where \a D arise as a side effect of other computation. For regular sitation use the end-point-only constructor.
         * @param beg begin point.
         * @param end end point.
         * @param dir unit direction vector.
         */
        LineSegmentND(const VectorType &beg, const VectorType &end, const VectorType dir) : LineSegmentND_common<2, Element, LineSegmentND>(beg, end, dir) {}

        //! Direct construction from coordinates of the end points.
        /*!
         * Convenience addition to regular end point constructor.
         * @param beg_x begin point x coordinate.
         * @param beg_y begin point y coordinate.
         * @param end_x end point x coordinate.
         * @param end_y end point y coordinate.
         */
        LineSegmentND(ElementType beg_x, ElementType beg_y, ElementType end_x, ElementType end_y) : LineSegmentND_common<2, Element, LineSegmentND>(VectorType(beg_x, beg_y), VectorType(end_x, end_y)) {}

        //! Normal vector.
        /*!
         * Normal vector given by counter-clockwise rotation od \a D.
         * @return normal vector.
         */
        VectorType normal() const
        {
            return VectorType(-LineSegmentND<2, Element>::int_dir[1], LineSegmentND<2, Element>::int_dir[0]);
        }

        //! Shortest Euclidean distance to origin.
        /*!
         *
         * @return the distance.
         */
        typename VectorType::DistanceType distanceToOrigin() const
        {
            return std::abs(this->int_beg.dot(normal()));
        } // c = -(ax + by)

        //! Set new begin point from coordinates.
        /*!
         *
         * @param beg_x x coordinate of the new begin point.
         * @param beg_y y coordinate of the new begin point.
         */
        void setBegin(Element beg_x, Element beg_y) { LineSegmentND_common<2, Element, LineSegmentND>::setBegin(VectorType(beg_x, beg_y)); }

        //! Set new end point from coordinates.
        /*!
         *
         * @param end_x x coordinate of the new end point.
         * @param end_y y coordinate of the new end point.
         */
        void setEnd(Element end_x, Element end_y) { LineSegmentND_common<2, Element, LineSegmentND>::setEnd(VectorType(end_x, end_y)); }

        //! Convenience alias to cropByHyperrect()
        bool cropByRect(const VectorType &corner1, const VectorType &corner2, Element &segment_beg_t, Element &segment_end_t) const
        {
            return cropByHyperRect(corner1, corner2, segment_beg_t, segment_end_t);
        }

        //! Convenience alias to cropByHyperrect()
        bool cropByRect(const VectorType &corner1, const VectorType &corner2, VectorType &segment_beg, VectorType &segment_end) const
        {
            return cropByHyperRect(corner1, corner2, segment_beg, segment_end);
        }

        //! Finds crossing of two line segments.
        /*!
         * Finds the crossing of \p first and \p second such that \a B1 + \p t_first(\a E1 - \a B1) = \a B2 + \p t_second(\a E2 - \a B2).
         * If the line segments are collinear, \p t_first and \p t_second are set to positive or negative infinite.
         * @param first first line segment.
         * @param second second line segment.
         * @param t_first first line segment parameter.
         * @param t_second second line segment parameter.
         * @return true if a unique crossing exists, false otherwise.
         */
        static bool getCrossing(const LineSegmentND<2, Element> &first, const LineSegmentND<2, Element> &second, Element &t_first, Element &t_second)
        {
            return getCrossing(first.int_beg, first.int_end, second.int_beg, second.int_end, t_first, t_second);
        }

        //! Finds crossing of two line segments.
        /*!
         * Does not modify \p crossing if a unique intersection does not exist.
         * @param first first first line segment.
         * @param second second second line segment.
         * @param crossing the point of intersection.
         * @return true if a unique crossing exists, false otherwise.
         */
        static bool getCrossing(const LineSegmentND<2, Element> &first, const LineSegmentND<2, Element> &second, VectorType &crossing)
        {
            return getCrossing(first.int_beg, first.int_end, second.int_beg, second.int_end, crossing);
        }

        //! Finds crossing of two line segments.
        /*!
         * Finds the crossing of two line segments such that \p first_beg + \p t_first(\a first_end - \a first_beg) = \a second_beg + \p t_second(\a second_end - \a second_beg).
         * If the line segments are collinear, \p t_first and \p t_second are set to positive or negative infinite.
         * @param first_beg first line segment begin point.
         * @param first_end first line segment end point.
         * @param second_beg second line segment begin point.
         * @param second_end second line segment end point.
         * @param t_first t_first first line segment parameter.
         * @param t_second t_second second line segment parameter.
         * @return true if a unique crossing exists, false otherwise.
         */
        static bool getCrossing(const VectorType &first_beg, const VectorType &first_end, const VectorType &second_beg, const VectorType &second_end, Element &t_first, Element &t_second)
        {
            VectorType v_first = first_end - first_beg;
            VectorType v_second = second_beg - second_end;
            VectorType v_begs = second_beg - first_beg;

            Element det = v_first[0] * v_second[1] - v_second[0] * v_first[1];
            t_first = v_begs[0] * v_second[1] - v_second[0] * v_begs[1];
            t_second = v_first[0] * v_begs[1] - v_begs[0] * v_first[1];

            if (det == 0.0f)
            {
                if (t_first > 0.0f)
                    t_first = std::numeric_limits<ElementType>::max();
                else if (t_first < 0.0f)
                    t_first = std::numeric_limits<ElementType>::lowest();

                if (t_second > 0.0f)
                    t_second = std::numeric_limits<ElementType>::max();
                else if (t_second < 0.0f)
                    t_second = std::numeric_limits<ElementType>::lowest();

                return false;
            }

            t_first /= det;
            t_second /= det;
            return true;
        }

        //! Finds crossing of two line segments.
        /*!
         * Does not modify \p crossing if a unique intersection does not exist.
         * @param first_beg first line segment begin point.
         * @param first_end first line segment end point.
         * @param second_beg second line segment begin point.
         * @param second_end second line segment end point.
         * @param crossing the point of intersection.
         * @return true if a unique crossing exists, false otherwise.
         */
        static bool getCrossing(const VectorType &first_beg, const VectorType &first_end, const VectorType &second_beg, const VectorType &second_end, VectorType &crossing)
        {
            VectorType v_first = first_end - first_beg;
            VectorType v_second = second_beg - second_end;
            VectorType v_begs = second_beg - first_beg;

            Element det = v_first[0] * v_second[1] - v_second[0] * v_first[1];
            if (det == 0.0f)
                return false;

            float t = (v_begs[0] * v_second[1] - v_second[0] * v_begs[1]) / det;
            crossing = first_beg + t * v_first;
            return true;
        }
    };


    template<typename Element>
    class LineSegmentND<3, Element> : public LineSegmentND_common<3, Element, LineSegmentND>
    {
    public:
        typedef typename LineSegmentND_common<3, Element, LineSegmentND>::ElementType ElementType;  //!< Base type for line segment coordinates.
        typedef typename LineSegmentND_common<3, Element, LineSegmentND>::VectorType VectorType;    //!< VectorND specialization for internal data.

        //! Default constructor. No initialization is performed.
        LineSegmentND() = default;

        //! Copy constructor.
        LineSegmentND(const LineSegmentND<3, Element> &ls) : LineSegmentND_common<3, Element, LineSegmentND>(ls) {}

        //! Construction from two end points.
        /*!
         *
         * @param beg begin point.
         * @param end end point.
         */
        LineSegmentND(const VectorType &beg, const VectorType &end) : LineSegmentND_common<3, Element, LineSegmentND>(beg, end) {}

        //! Construction from both end points and a corresponding direction vector.
        /*!
         * Care must be taken to to provide a true direction vector (\a E - \a B).normalized(). No validity checks are performed and member function may lead to unexpected results with wrong \a D.
         * This constructor is meant to reduce overhead in specific cases, where \a D arise as a side effect of other computation. For regular sitation use the end-point-only constructor.
         * @param beg begin point.
         * @param end end point.
         * @param dir unit direction vector.
         */
        LineSegmentND(const VectorType &beg, const VectorType &end, const VectorType dir) : LineSegmentND_common<3, Element, LineSegmentND>(beg, end, dir) {}

    };
}

#endif // ROBOTICTEMPLATELIBRARY_LINESEGMENTND_H
