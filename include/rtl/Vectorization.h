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

#ifndef ROBOTICTEMPLATELIBRARY_VECTORIZATION_H
#define ROBOTICTEMPLATELIBRARY_VECTORIZATION_H

#include <vector>
#include <list>
#include <map>
#include <utility>
#include <tuple>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

#include "rtl/Core.h"

#include "rtl/vect/ApproximationTlsLine2D.h"
#include "rtl/vect/ApproximationTlsLine3D.h"
#include "rtl/vect/ApproximationTlsPlane3D.h"
#include "rtl/vect/ExtractorChainFast.h"
#include "rtl/vect/ExtractorChainIncremental.h"
#include "rtl/vect/OptimizerContinuity2D.h"
#include "rtl/vect/OptimizerTotalError.h"
#include "rtl/vect/PostprocessorPolyline2D.h"
#include "rtl/vect/PostprocessorProjectEndpoints.h"
#include "rtl/vect/PrecArray.h"
#include "rtl/vect/PrecSums.h"
#include "rtl/vect/VectorizerPointElimination.h"

namespace rtl
{
    //! Two dimensional line extracting vectorizer.
    /*!
     * Vectorizer for extraction of total-least-squares line approximations in 2D using incremental extractor and projection postprocessor to trim approximations to line segments.
     * @tparam Element type for data element storage.
     * @tparam Compute type for precise computations.
     */
    template <typename Element, typename Compute>
    class VectorizerITLSProjections2D
    {
    public:
        typedef Element ElementType;                        //!< Type for data element storage.
        typedef Compute ComputeType;                        //!< Type for precise computations.
        typedef Vector2D<ElementType> VectorType;           //!< VectorND specialization for internal data.
        typedef LineSegment2D<ElementType> OutputType;      //!< Type of output geometrical object.
        typedef ApproximationTlsLine2D<ElementType, ComputeType> ApproximationType;     //!< Approximation type.
        typedef std::pair<size_t, size_t> IndexType;        //!< Type holding a pair of indices to an array.

        //! Default constructor.
        VectorizerITLSProjections2D() = default;

        //! Default destructor.
        ~VectorizerITLSProjections2D() = default;

        //! Sets maximal permitted standard deviation of point-approximation distances.
        /*!
         *
         * @param sigma new standard deviation.
         */
        void setSigma(ElementType sigma) { extractor.setSigma(sigma); }

        //! Extracted line approximations.
        /*!
         *
         * @return reference to internal buffer of extracted approximations.
         */
        [[nodiscard]] const std::vector<ApproximationType>& approximations() const { return int_lines; }

        //! Extracted line segments.
        /*!
         *
         * @return reference to internal buffer of extracted line segments.
         */
        [[nodiscard]] const std::vector<OutputType>& lineSegments() const { return postprocessor.output(); }

        //! Indices defining valid range for approximations() and lineSegments().
        /*!
         *
         * @return reference to internal buffer of valid range indices.
         */
        [[nodiscard]] const std::vector<IndexType>& indices() const { return int_indices; }

        //! Functor call for vectorization of an ordered point cloud.
        /*!
         * Process \p pts and generates output into internal buffers.
         * @param pts ordered point cloud.
         * @return true on success, false otherwise.
         */
        bool operator()(const std::vector<VectorType> &pts)
        {
            if(!extractor(pts, int_lines, int_indices))
                return false;
            return postprocessor(pts, int_lines, int_indices);
        }

    private:
        ExtractorChainIncremental<ApproximationType> extractor;
        PostprocessorProjectEndpoints<ApproximationType> postprocessor;

        std::vector<ApproximationType> int_lines;
        std::vector<IndexType> int_indices;
    };

    //! Fast two dimensional line extracting vectorizer.
    /*!
     * Vectorizer for extraction of total-least-squares line approximations in 2D using fast extractor and polyline postprocessor to trim approximations to line segments.
     * @tparam Element type for data element storage.
     * @tparam Compute type for precise computations.
     */
    template <typename Element, typename Compute>
    class VectorizerFTLSPolyline2D
    {
    public:
        typedef Element ElementType;                                //!< Type for data element storage.
        typedef Compute ComputeType;                                //!< Type for precise computations.
        typedef Vector2D<ElementType> VectorType;                   //!< VectorND specialization for internal data.
        typedef LineSegment2D<ElementType> OutputType;              //!< Type of output geometrical object.
        typedef PrecArray2D<ElementType, ComputeType> PrecArrayType;//!< PrecArray2D specialization.
        typedef ApproximationTlsLine2D<ElementType, ComputeType> ApproximationType;     //!< Approximation type.
        typedef std::pair<size_t, size_t> IndexType;                //!< Type holding a pair of indices to an array.

        //! Default constructor.
        VectorizerFTLSPolyline2D() = default;

        //! Default destructor.
        ~VectorizerFTLSPolyline2D() = default;

        //! Sets maximal permitted standard deviation of point-approximation distances.
        /*!
         *
         * @param sigma new standard deviation.
         */
        void setSigma(ElementType sigma) { extractor.setSigma(sigma); }

        //! Sets maximal permitted distance of intersection of the approximation lines from their neighbouring end points.
        /*!
         * \p delta should be circa 3-10 times larger that standard deviation used in preceding vectorization to avoid unnecessary splits of approximations
         * and maintain functionality of the optimization procedure.
         * @param delta new value of the maximal distance.
         */
        void setDelta(ElementType delta) { optimizer_continuity.setDelta(delta); }

        //! Change size of the precomputed sums array.
        /*!
         * Resizes the array to take required number of points to avoid unnecessary reallocation.
         * @param size number of points.
         */
        void setMaxSize(size_t size) { array.resize(size); }

        //! Extracted line approximations.
        /*!
         *
         * @return reference to internal buffer of extracted approximations.
         */
        [[nodiscard]] const std::vector<ApproximationType>& approximations() const { return int_lines; }

        //! Extracted line segments.
        /*!
         *
         * @return reference to internal buffer of extracted line segments.
         */
        [[nodiscard]] std::vector<OutputType> lineSegments() const { return postprocessor.lineSegments(); }

        //! Indices defining valid range for approximations() and lineSegments().
        /*!
         *
         * @return reference to internal buffer of valid range indices.
         */
        [[nodiscard]] const std::vector<IndexType>& indices() const { return int_indices; }

        //! Functor call for vectorization of an ordered point cloud.
        /*!
         * Process \p pts and generates output into internal buffers.
         * @param pts ordered point cloud.
         * @return true on success, false otherwise.
         */
        bool operator()(const std::vector<VectorType> &pts)
        {
            array.precompute(pts);
            if(!extractor(array, int_lines, int_indices))
                return false;
            if(!optimizer_continuity(pts, array, int_lines, int_indices))
                return false;
            return postprocessor(pts, int_lines, int_indices);
        }

    private:
        PrecArrayType array;
        ExtractorChainFast<PrecArrayType, ApproximationType> extractor;
        OptimizerContinuity2D<PrecArrayType, ApproximationType> optimizer_continuity;
        PostprocessorPolyline2D<ApproximationType> postprocessor;

        std::vector<ApproximationType> int_lines;
        std::vector<IndexType> int_indices;
    };

    //! Fast two dimensional line extracting vectorizer with global error optimization.
    /*!
     * Vectorizer for extraction of total-least-squares line approximations in 2D using fast extractor, global error optimization
     * and polyline postprocessor to trim approximations to line segments.
     * @tparam Element type for data element storage.
     * @tparam Compute type for precise computations.
     */
    template <typename Element, typename Compute>
    class VectorizerAFTLSPolyline2D
    {
    public:
        typedef Element ElementType;                                //!< Type for data element storage.
        typedef Compute ComputeType;                                //!< Type for precise computations.
        typedef Vector2D<ElementType> VectorType;                   //!< VectorND specialization for internal data.
        typedef LineSegment2D<ElementType> OutputType;              //!< Type of output geometrical object.
        typedef PrecArray2D<ElementType, ComputeType> PrecArrayType;//!< PrecArray2D specialization.
        typedef ApproximationTlsLine2D<ElementType, ComputeType> ApproximationType;     //!< Approximation type.
        typedef std::pair<size_t, size_t> IndexType;                //!< Type holding a pair of indices to an array.

        //! Default constructor.
        VectorizerAFTLSPolyline2D() = default;

        //! Default destructor.
        ~VectorizerAFTLSPolyline2D() = default;

        //! Change size of the precomputed sums array.
        /*!
         * Resizes the array to take required number of points to avoid unnecessary reallocation.
         * @param size number of points.
         */
        void setMaxSize(size_t size) { array.resize(size); }

        //! Sets maximal permitted standard deviation of point-approximation distances.
        /*!
         *
         * @param sigma new standard deviation.
         */
        void setSigma(ElementType sigma) { extractor.setSigma(sigma); }

        //! Sets initial shift of Nelder-Mead simplex vertices in global error optimization.
        /*!
         * Optimal value is usually between \a N/50 and \a N/500, where \a N is the number of processed points. Must be at least one, which is enforced by the function itself.
         * @param simplex_shift new initial shift.
         */
        void setSimplexShift(size_t simplex_shift) { optimizer_total_error.setSimplexShift(simplex_shift); }

        //! Sets maximal number of iterations of the global error optimization.
        /*!
         * Usually the optimization terminates much faster. This limit prevents rare endless loops in the program.
         * @param max_iterations new maximal number of iterations.
         */
        void setMaxIterations(size_t max_iterations) { optimizer_total_error.setMaxIterations(max_iterations); }

        //! Sets maximal permitted distance of intersection of the approximation lines from their neighbouring end points.
        /*!
         * \p delta should be circa 3-10 times larger that standard deviation used in preceding vectorization to avoid unnecessary splits of approximations
         * and maintain functionality of the optimization procedure.
         * @param delta new value of the maximal distance.
         */
        void setDelta(ElementType delta) { optimizer_continuity.setDelta(delta); }

        //! Extracted line approximations.
        /*!
         *
         * @return reference to internal buffer of extracted approximations.
         */
        [[nodiscard]] const std::vector<ApproximationType>& approximations() const { return int_lines; }

        //! Extracted line segments.
        /*!
         *
         * @return reference to internal buffer of extracted line segments.
         */
        [[nodiscard]] std::vector<OutputType> lineSegments() const { return postprocessor.lineSegments(); }

        //! Indices defining valid range for approximations() and lineSegments().
        /*!
         *
         * @return reference to internal buffer of valid range indices.
         */
        [[nodiscard]] const std::vector<IndexType>& indices() const { return int_indices; }

        //! Functor call for vectorization of an ordered point cloud.
        /*!
         * Process \p pts and generates output into internal buffers.
         * @param pts ordered point cloud.
         * @return true on success, false otherwise.
         */
        bool operator()(const std::vector<VectorType> &pts)
        {
            array.precompute(pts);
            if(!extractor(array, int_lines, int_indices))
                return false;
            if(!optimizer_total_error(pts, array, int_lines, int_indices))
                return false;
            if(!optimizer_continuity(pts, array, int_lines, int_indices))
                return false;
            return postprocessor(pts, int_lines, int_indices);
        }

    private:
        PrecArrayType array;
        ExtractorChainFast<PrecArrayType, ApproximationType> extractor;
        OptimizerTotalError<PrecArrayType, ApproximationType> optimizer_total_error;
        OptimizerContinuity2D<PrecArrayType, ApproximationType> optimizer_continuity;
        PostprocessorPolyline2D<ApproximationType> postprocessor;

        std::vector<ApproximationType> int_lines;
        std::vector<IndexType> int_indices;
    };

    //! Three dimensional line extracting vectorizer.
    /*!
     * Vectorizer for extraction of total-least-squares line approximations in 3D using incremental extractor and projection postprocessor to trim approximations to line segments.
     * @tparam Element type for data element storage.
     * @tparam Compute type for precise computations.
     */
    template <typename Element, typename Compute>
    class VectorizerITLSProjections3D
    {
    public:
        typedef Element ElementType;                    //!< Type for data element storage.
        typedef Compute ComputeType;                    //!< Type for precise computations.
        typedef Vector3D<ElementType> VectorType;       //!< VectorND specialization for internal data.
        typedef LineSegment3D<ElementType> OutputType;  //!< Type of output geometrical object.
        typedef ApproximationTlsLine3D<ElementType, ComputeType> ApproximationType; //!< Approximation type.
        typedef std::pair<size_t, size_t> IndexType;    //!< Type holding a pair of indices to an array.

        //! Default constructor.
        VectorizerITLSProjections3D() = default;

        //! Default destructor.
        ~VectorizerITLSProjections3D() = default;

        //! Sets maximal permitted standard deviation of point-approximation distances.
        /*!
         *
         * @param sigma new standard deviation.
         */
        void setSigma(ElementType sigma) { extractor.setSigma(sigma); }

        //! Extracted line approximations.
        /*!
         *
         * @return reference to internal buffer of extracted approximations.
         */
        [[nodiscard]] const std::vector<ApproximationType>& approximations() const { return int_lines; }

        //! Extracted line segments.
        /*!
         *
         * @return reference to internal buffer of extracted line segments.
         */
        [[nodiscard]] const std::vector<OutputType>& lineSegments() const { return postprocessor.output(); }

        //! Indices defining valid range for approximations() and lineSegments().
        /*!
         *
         * @return reference to internal buffer of valid range indices.
         */
        [[nodiscard]] const std::vector<IndexType>& indices() const { return int_indices; }

        //! Functor call for vectorization of an ordered point cloud.
        /*!
         * Process \p pts and generates output into internal buffers.
         * @param pts ordered point cloud.
         * @return true on success, false otherwise.
         */
        bool operator()(const std::vector<VectorType> &pts)
        {
            if(!extractor(pts, int_lines, int_indices))
                return false;
            return postprocessor(pts, int_lines, int_indices);
        }

    private:
        ExtractorChainIncremental<ApproximationType> extractor;
        PostprocessorProjectEndpoints<ApproximationType> postprocessor;

        std::vector<ApproximationType> int_lines;
        std::vector<IndexType> int_indices;
    };

    //! Fast three dimensional line extracting vectorizer.
    /*!
     * Vectorizer for extraction of total-least-squares line approximations in 3D using fast extractor and projection postprocessor to trim approximations to line segments.
     * @tparam Element type for data element storage.
     * @tparam Compute type for precise computations.
     */
    template <typename Element, typename Compute>
    class VectorizerFTLSProjections3D
    {
    public:
        typedef Element ElementType;                                    //!< Type for data element storage.
        typedef Compute ComputeType;                                    //!< Type for precise computations.
        typedef Vector3D<ElementType> VectorType;                       //!< VectorND specialization for internal data.
        typedef LineSegment3D<ElementType> OutputType;                  //!< Type of output geometrical object.
        typedef PrecArray3D<ElementType, ComputeType> PrecArrayType;    //!< PrecArray3D specialization.
        typedef ApproximationTlsLine3D<ElementType, ComputeType> ApproximationType;     //!< Approximation type.
        typedef std::pair<size_t, size_t> IndexType;                    //!< Type holding a pair of indices to an array.

        //! Default constructor.
        VectorizerFTLSProjections3D() = default;

        //! Default destructor.
        ~VectorizerFTLSProjections3D() = default;

        //! Sets maximal permitted standard deviation of point-approximation distances.
        /*!
         *
         * @param sigma new standard deviation.
         */
        void setSigma(ElementType sigma) { extractor.setSigma(sigma); }

        //! Change size of the precomputed sums array.
        /*!
         * Resizes the array to take required number of points to avoid unnecessary reallocation.
         * @param size number of points.
         */
        void setMaxSize(size_t size) { array.resize(size); }

        //! Extracted line approximations.
        /*!
         *
         * @return reference to internal buffer of extracted approximations.
         */
        [[nodiscard]] const std::vector<ApproximationType>& approximations() const { return int_lines; }

        //! Extracted line segments.
        /*!
         *
         * @return reference to internal buffer of extracted line segments.
         */
        [[nodiscard]] const std::vector<OutputType>& lineSegments() const { return postprocessor.output(); }

        //! Indices defining valid range for approximations() and lineSegments().
        /*!
         *
         * @return reference to internal buffer of valid range indices.
         */
        [[nodiscard]] const std::vector<IndexType>& indices() const { return int_indices; }

        //! Functor call for vectorization of an ordered point cloud.
        /*!
         * Process \p pts and generates output into internal buffers.
         * @param pts ordered point cloud.
         * @return true on success, false otherwise.
         */
        bool operator()(const std::vector<VectorType> &pts)
        {
            array.precompute(pts);
            if(!extractor(array, int_lines, int_indices))
                return false;
            return postprocessor(pts, int_lines, int_indices);
        }

    private:
        PrecArrayType array;
        ExtractorChainFast<PrecArrayType, ApproximationType> extractor;
        PostprocessorProjectEndpoints<ApproximationType> postprocessor;

        std::vector<ApproximationType> int_lines;
        std::vector<IndexType> int_indices;
    };

    //! Fast three dimensional line extracting vectorizer with global error optimization.
    /*!
     * Vectorizer for extraction of total-least-squares line approximations in 3D using fast extractor, global error optimization
     * and projection postprocessor to trim approximations to line segments.
     * @tparam Element type for data element storage.
     * @tparam Compute type for precise computations.
     */
    template <typename Element, typename Compute>
    class VectorizerAFTLSProjections3D
    {
    public:
        typedef Element ElementType;                                //!< Type for data element storage.
        typedef Compute ComputeType;                                //!< Type for precise computations.
        typedef Vector3D<ElementType> VectorType;                   //!< VectorND specialization for internal data.
        typedef LineSegment3D<ElementType> LineSegmentType;         //!< Type of output geometrical object.
        typedef PrecArray3D<ElementType, ComputeType> PrecArrayType;//!< PrecArray3D specialization.
        typedef ApproximationTlsLine3D<ElementType, ComputeType> ApproximationType;     //!< Approximation type.
        typedef std::pair<size_t, size_t> IndexType;                //!< Type holding a pair of indices to an array.

        //! Default constructor.
        VectorizerAFTLSProjections3D() = default;

        //! Default destructor.
        ~VectorizerAFTLSProjections3D() = default;

        //! Change size of the precomputed sums array.
        /*!
         * Resizes the array to take required number of points to avoid unnecessary reallocation.
         * @param size number of points.
         */
        void setMaxSize(size_t size) { array.resize(size); }

        //! Sets maximal permitted standard deviation of point-approximation distances.
        /*!
         *
         * @param sigma new standard deviation.
         */
        void setSigma(ElementType sigma) { extractor.setSigma(sigma); }

        //! Sets maximal number of iterations of the global error optimization.
        /*!
         * Usually the optimization terminates much faster. This limit prevents rare endless loops in the program.
         * @param max_iterations new maximal number of iterations.
         */
        void setSimplexShift(size_t simplex_shift) { optimizer_total_error.setSimplexShift(simplex_shift); }

        //! Sets maximal permitted distance of intersection of the approximation lines from their neighbouring end points.
        /*!
         * \p delta should be circa 3-10 times larger that standard deviation used in preceding vectorization to avoid unnecessary splits of approximations
         * and maintain functionality of the optimization procedure.
         * @param delta new value of the maximal distance.
         */
        void setMaxIterations(size_t max_iterations) { optimizer_total_error.setMaxIterations(max_iterations); }

        //! Extracted line approximations.
        /*!
         *
         * @return reference to internal buffer of extracted approximations.
         */
        [[nodiscard]] const std::vector<ApproximationType>& approximations() const { return int_lines; }

        //! Extracted line segments.
        /*!
         *
         * @return reference to internal buffer of extracted line segments.
         */
        [[nodiscard]] const std::vector<LineSegmentType>& lineSegments() const { return postprocessor.output(); }

        //! Indices defining valid range for approximations() and lineSegments().
        /*!
         *
         * @return reference to internal buffer of valid range indices.
         */
        [[nodiscard]] const std::vector<IndexType>& indices() const { return int_indices; }

        //! Functor call for vectorization of an ordered point cloud.
        /*!
         * Process \p pts and generates output into internal buffers.
         * @param pts ordered point cloud.
         * @return true on success, false otherwise.
         */
        bool operator()(const std::vector<VectorType> &pts)
        {
            array.precompute(pts);
            if(!extractor(array, int_lines, int_indices))
                return false;
            if(!optimizer_total_error(pts, array, int_lines, int_indices))
                return false;
            return postprocessor(pts, int_lines, int_indices);
        }

    private:
        PrecArrayType array;
        ExtractorChainFast<PrecArrayType, ApproximationType> extractor;
        OptimizerTotalError<PrecArrayType, ApproximationType> optimizer_total_error;
        PostprocessorProjectEndpoints<ApproximationType> postprocessor;

        std::vector<ApproximationType> int_lines;
        std::vector<IndexType> int_indices;
    };

    //! Fast three dimensional plane extracting vectorizer with global error optimization.
    /*!
     * Vectorizer for extraction of total-least-squares plane approximations in 3D using fast extractor, global error optimization
     * and projection postprocessor to trim approximations to polygons.
     * @tparam Element type for data element storage.
     * @tparam Compute type for precise computations.
     */
    template <typename Element, typename Compute>
    class VectorizerAFTLSPlaneProjections3D
    {
    public:
        typedef Element ElementType;                                //!< Type for data element storage.
        typedef Compute ComputeType;                                //!< Type for precise computations.
        typedef Vector3D<ElementType> VectorType;                   //!< VectorND specialization for internal data.
        typedef Polygon3D<ElementType> OutputType;                  //!< Type of output geometrical object.
        typedef PrecArray3D<ElementType, ComputeType> PrecArrayType;//!< PrecArray3D specialization.
        typedef ApproximationTlsPlane3D<ElementType, ComputeType> ApproximationType;     //!< Approximation type.
        typedef std::pair<size_t, size_t> IndexType;                //!< Type holding a pair of indices to an array.

        //! Default constructor.
        VectorizerAFTLSPlaneProjections3D() = default;

        //! Default destructor.
        ~VectorizerAFTLSPlaneProjections3D() = default;

        //! Change size of the precomputed sums array.
        /*!
         * Resizes the array to take required number of points to avoid unnecessary reallocation.
         * @param size number of points.
         */
        void setMaxSize(size_t size) { array.resize(size); }

        //! Sets maximal permitted standard deviation of point-approximation distances.
        /*!
         *
         * @param sigma new standard deviation.
         */
        void setSigma(ElementType sigma) { extractor.setSigma(sigma); }

        //! Sets maximal number of iterations of the global error optimization.
        /*!
         * Usually the optimization terminates much faster. This limit prevents rare endless loops in the program.
         * @param max_iterations new maximal number of iterations.
         */
        void setSimplexShift(size_t simplex_shift) { optimizer_total_error.setSimplexShift(simplex_shift); }

        //! Sets maximal permitted distance of intersection of the approximation lines from their neighbouring end points.
        /*!
         * \p delta should be circa 3-10 times larger that standard deviation used in preceding vectorization to avoid unnecessary splits of approximations
         * and maintain functionality of the optimization procedure.
         * @param delta new value of the maximal distance.
         */
        void setMaxIterations(size_t max_iterations) { optimizer_total_error.setMaxIterations(max_iterations); }

        //! Extracted line approximations.
        /*!
         *
         * @return reference to internal buffer of extracted approximations.
         */
        [[nodiscard]] const std::vector<ApproximationType>& approximations() const { return int_lines; }

        //! Extracted polygons.
        /*!
         *
         * @return reference to internal buffer of extracted polygons.
         */
        [[nodiscard]] const std::vector<OutputType>& polygons() const { return postprocessor.output(); }

        //! Indices defining valid range for approximations() and polygons().
        /*!
         *
         * @return reference to internal buffer of valid range indices.
         */
        [[nodiscard]] const std::vector<IndexType>& indices() const { return int_indices; }

        //! Functor call for vectorization of an ordered point cloud.
        /*!
         * Process \p pts and generates output into internal buffers.
         * @param pts ordered point cloud.
         * @return true on success, false otherwise.
         */
        bool operator()(const std::vector<VectorType> &pts)
        {
            array.precompute(pts);
            if(!extractor(array, int_lines, int_indices))
                return false;
            if(!optimizer_total_error(pts, array, int_lines, int_indices))
                return false;
            return postprocessor(pts, int_lines, int_indices);
        }

    private:
        PrecArrayType array;
        ExtractorChainFast<PrecArrayType, ApproximationType> extractor;
        OptimizerTotalError<PrecArrayType, ApproximationType> optimizer_total_error;
        PostprocessorProjectEndpoints<ApproximationType> postprocessor;

        std::vector<ApproximationType> int_lines;
        std::vector<IndexType> int_indices;
    };

    using VectorizerDouglasPeucker2f = VectorizerDouglasPeuckerND<2, float>;
    using VectorizerDouglasPeucker2d = VectorizerDouglasPeuckerND<2, double>;
    using VectorizerDouglasPeucker3f = VectorizerDouglasPeuckerND<3, float>;
    using VectorizerDouglasPeucker3d = VectorizerDouglasPeuckerND<3, double>;

    using VectorizerReumannWitkam2f = VectorizerReumannWitkamND<2, float>;
    using VectorizerReumannWitkam2d = VectorizerReumannWitkamND<2, double>;
    using VectorizerReumannWitkam3f = VectorizerReumannWitkamND<3, float>;
    using VectorizerReumannWitkam3d = VectorizerReumannWitkamND<3, double>;
}

#endif // ROBOTICTEMPLATELIBRARY_VECTORIZATION_H
