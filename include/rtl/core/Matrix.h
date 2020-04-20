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

#ifndef ROBOTICTEMPLATELIBRARY_MATRIX_H
#define ROBOTICTEMPLATELIBRARY_MATRIX_H

#include <type_traits>
#include <complex>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

#include "rtl/core/VectorND.h"

namespace rtl
{
    //! Template for general Matrices of both static and dynamic nature.
    /*!
     * Static matrices are constructed using positive integer in place of \p rows and \p cols, while dynamic use Eigen::Dynamic at these places.
     * @tparam rows number of matrix rows specified at compile time, or Eigen::Dynamic.
     * @tparam cols number of matrix columns specified at compile time, or Eigen::Dynamic.
     * @tparam Element base type of matrix elements.
     */
    template<int rows, int cols, typename Element>
    class Matrix
    {
    public:
        typedef Element ElementType;        //!< base type for vector elements.
        typedef Element DistanceType;       //!< return type of the distance function.
        typedef Eigen::Matrix<Element, rows, cols> EigenType;       //!< type of the underlying Eigen object.

        template<int, int, typename> friend
        class Matrix;

        //! Default constructor. No initialization is performed.
        Matrix() = default;

        //! Compile-time constriction of dynamic matrix of given size.
        /*!
         * Only available for matrices with both rows and columns set to be Eigen::Dynamic.
         * @param r number of rows required.
         * @param c number of columns required.
         */
        Matrix(int r, int c) : int_matrix(r, c)
        {
            static_assert(rows == Eigen::Dynamic && cols == Eigen::Dynamic, "Only dynamic matrices can be given initial sum_nr.");
        }

        //! Copy  constructor.
        Matrix(const Matrix<rows, cols, Element> &m)
        {
            int_matrix = m.int_matrix;
        }

        //! Construction from the underlying EigenType.
        explicit Matrix(const EigenType &em)
        {
            int_matrix = em;
        }

        //! Default destructor.
        ~Matrix() = default;

        //! Casting to different ElementType.
        template<typename NewElement>
        Matrix<rows, cols, NewElement> cast() const
        {
            return Matrix<rows, cols, NewElement>(int_matrix.template cast<NewElement>());
        }

        //! Reference to underlying Eigen data.
        EigenType &data()
        {
            return int_matrix;
        }

        //! Const reference to underlying Eigen data.
        const EigenType &data() const
        {
            return int_matrix;
        }

        //! Tests whether Not-a-Numbers are present in the matrix.
        /*!
         * Only works for floating point ElementType, since no other type is guaranteed to have NaN representation.
         * @return true if at least one NaN is present, false otherwise.
         */
        [[nodiscard]] bool hasNaN() const
        {
            return int_matrix.hasNaN();
        }

        //! Return copy of the element on r-th row and c-th column.
        /*!
         *
         * @param r row where the element of interest is.
         * @param c column where the element of interest is.
         * @return copy of the element on r-th row and c-th column.
         */
        Element getElement(size_t r, size_t c) const
        {
            return int_matrix(r, c);
        }

        //! Return new vector with elements copied from \p r -th row of the matrix.
        /*!
         *
         * @param r row to be extracted.
         * @return Vector containing copy of the r-th row.
         */
        VectorND<cols, Element> getRow(size_t r) const
        {
            static_assert(cols != Eigen::Dynamic, "Only static matrices can work with static sized vectors.");
            return VectorND<cols, Element>(int_matrix.row(r));
        }

        //! Return new vector with elements copied from \p c -th column of the matrix.
        /*!
         *
         * @param c column to be extracted.
         * @return Vector containing copy of the c-th column.
         */
        VectorND<rows, Element> getColumn(size_t c) const
        {
            static_assert(rows != Eigen::Dynamic, "Only static matrices can work with static sized vectors.");
            return VectorND<rows, Element>(int_matrix.col(c));
        }

        //! Sets the element on r-th row and c-th column to \p value.
        /*!
         *
         * @param r row where the element to be set is.
         * @param c column where the element to be set is.
         * @param value new value of the element.
         */
        void setElement(size_t r, size_t c, Element value)
        {
            int_matrix(r, c) = value;
        }

        //! Sets r-th row of the matrix to \p row.
        /*!
         *
         * @param r row to be set.
         * @param row vector of new values.
         */
        void setRow(size_t r, const VectorND<cols, Element> &row)
        {
            static_assert(cols != Eigen::Dynamic, "Only static matrices can work with static sized vectors.");
            int_matrix.row(r) = row.data();
        }

        //! Sets c-th column of the matrix to \p column.
        /*!
         *
         * @param c column to be set.
         * @param column vector of new values.
         */
        void setColumn(size_t c, const VectorND<rows, Element> &column)
        {
            static_assert(rows != Eigen::Dynamic, "Only static matrices can work with static sized vectors.");
            int_matrix.col(c) = column.data();
        }

        //! Element access operator.
        /*!
         * Reference to given element.
         * @param r row of the element of interest.
         * @param c column of the element of interest.
         * @return reference to the element on \p r -th row and \p c -th column.
         */
        Element &operator()(size_t r, size_t c)
        {
            return int_matrix(r, c);
        }

        //! Element access operator for reading.
        /*!
         * Reference to given element.
         * @param r row of the element of interest.
         * @param c column of the element of interest.
         * @return constant reference to the element on \p r -th row and \p c -th column.
         */
        const Element &operator()(size_t r, size_t c) const
        {
            return int_matrix(r, c);
        }

        //! Assignment operator.
        /*!
         * Copies content of \p m to *this.
         * @param m the matrix with the data to be copied.
         * @return reference to *this.
         */
        Matrix<rows, cols, Element> &operator=(const Matrix<rows, cols, Element> &m)
        {
            int_matrix = m.int_matrix;
            return *this;
        }

        //! Assignment operator for underlying Eigen type.
        /*!
         * Copies content of \p em to *this.
         * @param em the Eigen matrix with the data to be copied.
         * @return reference to *this.
         */
        Matrix<rows, cols, Element> &operator=(const EigenType &em)
        {
            int_matrix = em;
            return *this;
        }

        //! Equality test operator.
        /*!
         *
         * @param m matrix to be tested.
         * @return true if all elements exactly match the counterparts on the same position, false otherwise.
         */
        bool operator==(const Matrix<rows, cols, Element> &m) const
        {
            return int_matrix == m.int_matrix;
        }

        //! Inequality test operator.
        /*!
         *
         * @param m matrix to be tested.
         * @return true if at least one element does not match its counterpart on the same position, false otherwise.
         */
        bool operator!=(const Matrix<rows, cols, Element> &m) const
        {
            return int_matrix != m.int_matrix;
        }

        //! Multiplication by a vector from the right.
        /*!
         * Only available for static sized matrices.
         * @param vector right side operand.
         * @return new vector with the result of multiplication.
         */
        VectorND<cols, Element> operator*(const VectorND<cols, Element> &vector) const
        {
            static_assert(cols != Eigen::Dynamic, "Only static matrices can work with static sized vectors.");
            return VectorND<cols, Element>(int_matrix * vector.elements);
        }

        //! Matrix-matrix multiplication.
        /*!
         * Regular matrix-matrix product.
         * @tparam op_cols number of columns of the right side matrix.
         * @param matrix the right side operand.
         * @return new matrix with the result of multiplication.
         */
        template<int op_cols> Matrix<rows, op_cols, Element> operator*(const Matrix<cols, op_cols, Element> &matrix) const
        {
            return Matrix<rows, op_cols, Element>(int_matrix * matrix.int_matrix);
        }

        //! Scalar multiplication.
        /*!
         * Regular matrix-scalar multiplication.
         * @param scalar the multiplicator.
         * @return new matrix with the result of the computation.
         */
        Matrix<rows, cols, Element> operator*(const Element &scalar) const
        {
            return Matrix<rows, cols, Element>(int_matrix * scalar);
        }

        //! Scalar division.
        /*!
         * Regular division by a scalar factor.
         * @param scalar the divisior.
         * @return new matrix with the result of division.
         */
        Matrix<rows, cols, Element> operator/(const Element &scalar) const
        {
            return Matrix<rows, cols, Element>(int_matrix / scalar);
        }

        //! In-place matrix-matrix multiplication.
        /*!
         * Restricted to square matricies, because the result of in-place matrix multiplication must exactly fit into memory of *this.
         * @param matrix the right side operand.
         * @return reference to *this.
         */
        Matrix<rows, cols, Element> &operator*=(const Matrix<rows, cols, Element> &matrix)
        {
            static_assert(rows == cols, "MatrixS::operator*= requires square matrices of the same sum_nr.");
            int_matrix *= matrix.int_matrix;
            return *this;
        }

        //! In-place scalar multiplication.
        /*!
         *
         * @param scalar the multiplier.
         * @return reference to *this.
         */
        Matrix<rows, cols, Element> &operator*=(const Element &scalar)
        {
            int_matrix *= scalar;
            return *this;
        }

        //! In-place scalar division.
        /*!
         *
         * @param scalar the divisor.
         * @return reference to *this.
         */
        Matrix<rows, cols, Element> &operator/=(const Element &scalar)
        {
            int_matrix /= scalar;
            return *this;
        }

        //! Matrix addition.
        /*!
         *
         * @param matrix matrix to be added to *this.
         * @return new matrix with the result of addition.
         */
        Matrix<rows, cols, Element> operator+(const Matrix<rows, cols, Element> &matrix) const
        {
            return Matrix<rows, cols, Element>(int_matrix + matrix.int_matrix);
        }

        //! In-place matrix addition.
        /*!
         *
         * @param matrix matrix to be added to *this.
         * @return reference to *this.
         */
        Matrix<rows, cols, Element> &operator+=(const Matrix<rows, cols, Element> &matrix)
        {
            int_matrix += matrix.int_matrix;
            return *this;
        }

        //! Matrix subtraction.
        /*!
         *
         * @param matrix matrix to be subtracted from *this.
         * @return new matrix with the result of subtraction.
         */
        Matrix<rows, cols, Element> operator-(const Matrix<rows, cols, Element> &matrix) const
        {
            return Matrix<rows, cols, Element>(int_matrix - matrix.int_matrix);
        }

        //! In-place matrix subtracion.
        /*!
         *
         * @param matrix matrix to be subtracted from *this.
         * @return reference to *this.
         */
        Matrix<rows, cols, Element> &operator-=(const Matrix<rows, cols, Element> &matrix)
        {
            int_matrix -= matrix.int_matrix;
            return *this;
        }

        //! Unary minus for inversion of all elements.
        /*!
         * Corresponds to multiplication by -1.
         * @return new matrix with all elements set to their negative.
         */
        Matrix<rows, cols, Element> operator-() const
        {
            return Matrix<rows, cols, Element>(-int_matrix);
        }

        //! In-place transposition of the matrix.
        /*!
         * Only available for square matrices.
         */
        void transpose()
        {
            static_assert(rows == cols, "MatrixS::transpose Only square matrices can be transposed in place.");
            int_matrix.transposeInPlace();
        }

        //! Transposition of the matrix.
        /*!
         *
         * @return new transposed copy of *this.
         */
        Matrix<cols, rows, Element> transposed() const
        {
            return Matrix<cols, rows, Element>(int_matrix.transpose());
        }

        //! In-place inversion of the matrix.
        /*!
         * Only available for square matrices.
         */
        void invert()
        {
            static_assert(rows == cols, "MatrixS::invert Only square matrices can be inverted.");
            int_matrix = int_matrix.inverse().eval();
        }

        //! Inverse of the matrix.
        /*!
         * Only available for square matrices.
         * @return new matrix with inversion of *this.
         */
        Matrix<cols, rows, Element> inverted() const
        {
            static_assert(rows == cols, "MatrixS::invert Only square matrices can be inverted.");
            return Matrix<cols, rows, Element>(int_matrix.inverse());
        }

        //! Determinant of the matrix.
        /*!
         *
         * @return determinant of the matrix.
         */
        Element determinant() const
        {
            return int_matrix.determinant();
        }

        //! Trace of the matrix.
        /*!
         *
         * @return trace of the matrix.
         */
        Element trace() const
        {
            return int_matrix.trace();
        }

        //! Eigenvalues of the matrix.
        /*!
         * Eigenvalues are repeated according to their algebraic multiplicity and are not sorted in any defined order.
         * @return vector of std::complex eigenvalues.
         */
        VectorND<rows, std::complex<Element>> eigenvalues() const
        {
            static_assert(rows == cols, "MatrixS::eigenvalues Only square matrices can have eigenvalues.");
            return VectorND<rows, std::complex<Element>>(int_matrix.eigenvalues());
        }

        //! Eigenvectors of the matrix.
        /*!
         * Eigenvectors order correspond to the order of the eigenvalues obtained through eigenvalues().-
         * @return matrix whose std::complex columns are the eigenvectors.
         */
        Matrix<cols, rows, std::complex<Element>> eigenvectors() const
        {
            static_assert(rows == cols, "MatrixS::eigenvectors Only square matrices can have eigenvectors.");
            Eigen::EigenSolver<EigenType> es(int_matrix);
            return Matrix<cols, rows, std::complex<Element>>(es.eigenvectors());
        }

        //! Number of rows.
        /*!
         *
         * @return number of rows, for dynamic matrices returns Eigen::Dynamic.
         */
        static constexpr int rowNr()
        {
            return rows;
        }

        //! Number of columns.
        /*!
         *
         * @return number of columns, for dynamic matrices returns Eigen::Dynamic.
         */
        static constexpr  int colNr()
        {
            return cols;
        }

        //! Zero initialized matrix of given type.
        /*!
         *
         * @return zero initialized matrix.
         */
        static Matrix<rows, cols, Element> zeros()
        {
            Matrix<rows, cols, Element> ret;
            ret.int_matrix = EigenType::Zero();
            return ret;
        }

        //! Initialized matrix of given type with all elements equal to one.
        /*!
         *
         * @return ones initialized matrix.
         */
        static Matrix<rows, cols, Element> ones()
        {
            Matrix<rows, cols, Element> ret;
            ret.int_matrix = EigenType::Constant(static_cast<Element>(1));
            return ret;
        }

        //! Identity initialized matrix of given type.
        /*!
         *
         * @return identity matrix.
         */
        static Matrix<rows, cols, Element> identity()
        {
            Matrix<rows, cols, Element> ret;
            ret.int_matrix = EigenType::Identity();
            return ret;
        }

        //! Initialized matrix of given type with all elements equal to not-a-number.
        /*!
         *
         * @return NaN initialized matrix.
         */
        static Matrix<rows, cols, Element> nan()
        {
            static_assert(std::is_floating_point<Element>::value, "Only floating point types support NaN.");
            Matrix<rows, cols, Element> ret;
            ret.int_matrix = EigenType::Constant(std::numeric_limits<Element>::quiet_NaN());
            return ret;
        }

        //! Frobenius norm of the difference of two matrices.
        /*!
         * Computes the difference \p m1 - \p m2 and returns square root of the sum of squares of all elements of that difference.
         * @param m1 first operand.
         * @param m2 second operand.
         * @return the Frobenius norm.
         */
        static DistanceType distance(const Matrix<rows, cols, Element> &m1, const Matrix<rows, cols, Element> &m2)
        {
            return (m1 - m2).data().norm();
        }

        //! Squared Frobenius norm of the difference of two matrices.
        /*!
         * Computes the difference \p m1 - \p m2 and returns the sum of squares of all elements of that difference.
         * @param m1 first operand.
         * @param m2 second operand.
         * @return the squared Frobenius norm.
         */
        static DistanceType distanceSquared(const Matrix<rows, cols, Element> &m1, const Matrix<rows, cols, Element> &m2)
        {
            return (m1 - m2).data().squaredNorm();
        }

        //! Return a new matrix with all elements initialized by the user-supplied random generator.
        /*!
         * Randomness is fully specified by the callable object \p el_rnd_gen, in fact it does not have to be random at all.
         * @tparam ElRndSrc type of the random generating object.
         * @param el_rnd_gen callable object returning ElementType on invocation.
         * @return a new vector initialized by \p el_rnd_gen.
         */
        template<class ElRndSrc>
        static Matrix<rows, cols, Element> random(const ElRndSrc &el_rnd_gen)
        {
            EigenType et;
            et = et.unaryExpr([&el_rnd_gen](Element) { return (Element) el_rnd_gen(); });
            return Matrix<rows, cols, Element>(et);
        }


    private:
        EigenType int_matrix;
    };

    //! Multiplication by a vector from the left.
    /*!
     * Only available for static sized matrices. The vectors are treated as if they were transposed (row).
     * @param vector left side operand.
     * @param matrix right side operand.
     * @return new vector with the result of multiplication.
     */
    template<int rows, int cols, typename E>
    VectorND<cols, E> operator*(const VectorND<rows, E> &vector, const Matrix<rows, cols, E> &matrix)
    {
        return VectorND<cols, E>(vector.data().transpose() * matrix.data());
    }

    //! Multiplication by a scalar from the left.
    /*!
     *
     * @param scalar left side operand.
     * @param matrix right side operand.
     * @return new matrix with the result of multiplication.
     */
    template<int rows, int cols, typename E>
    Matrix<rows, cols, E> operator*(const E &scalar, const Matrix<rows, cols, E> &matrix)
    {
        return matrix * scalar;
    }
}


#endif // ROBOTICTEMPLATELIBRARY_MATRIX_H
