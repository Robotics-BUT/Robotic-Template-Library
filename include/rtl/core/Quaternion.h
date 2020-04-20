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

#ifndef ROBOTICTEMPLATELIBRARY_QUATERNION_H
#define ROBOTICTEMPLATELIBRARY_QUATERNION_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "rtl/core/VectorND.h"
#include "rtl/core/Matrix.h"

namespace rtl
{
    //! Quaternion template for 3D rotations and regular computation.
    /*!
     * The template is mainly oriented on rotational stuff, but supports basic quaternion arithmetic as well.
     * @tparam Element base type of underlying data.
     */
    template<typename Element>
    class Quaternion
    {
    public:
        typedef Element ElementType;                        //!< Base type of underlying data.
        typedef VectorND<3, Element> VectorType;            //!< VectorND specialization for vector-requiring interface.
        typedef Eigen::Quaternion<ElementType> EigenType;   //!< type of the underlying Eigen object.

        //! Default constructor. No initialization is performed.
        Quaternion() : int_quat() {}

        //! Construction from underlying EigenType.
        explicit Quaternion(const EigenType &eq) : int_quat(eq) {}

        //! Element-wise construction.
        /*!
         * Initialization of the quaternion in the form: \a q = \a w + \a xi + \a yj + \a zk.
         * @param w real component.
         * @param x first imaginary component (\a xi).
         * @param y second imaginary component (\a yj).
         * @param z third imaginary component (\a zk).
         */
        Quaternion(ElementType w, ElementType x, ElementType y, ElementType z) : int_quat(w, x, y, z) {}

        //! Copy constructor.
        Quaternion(const Quaternion &q) = default;

        //! Angle+axis construction.
        /*!
         * Constructor for rotation computations. Ensures unity of the resulting quaternion.
         * @param angle angle in radians.
         * @param axis axis of rotation.
         */
        Quaternion(float angle, const VectorType &axis) : int_quat(Eigen::AngleAxis<ElementType>(angle, axis.data()))
        {
        }

        //! Two vector constructor.
        /*!
         * Finds a quaternion, which would turn \p v_from into the same direction as the \p v_to has using the shortest path.
         * Ensures unity of the resulting quaternion.
         * @param v_from vector to be rotated.
         * @param v_to required direction of \p v_from after the quaternion rotation is applied.
         */
        Quaternion(const VectorType &v_from, const VectorType &v_to)
        {
            int_quat = EigenType::FromTwoVectors(v_from.data(), v_to.data());
        }

        //! Roll-pitch-yaw rotation quaternion constructor.
        /*!
         * Constructs rotational quaternion using the RPY convention. Ensures unity of the resulting quaternion.
         * @param roll rotation around \a x axis.
         * @param pitch rotation around \a y axis.
         * @param yaw rotation around \a z axis.
         */
        Quaternion(ElementType roll, ElementType pitch, ElementType yaw)
        {
            int_quat = Eigen::AngleAxis(roll, Eigen::Matrix<ElementType, 3, 1>(1, 0, 0))
                    * Eigen::AngleAxis(pitch, Eigen::Matrix<ElementType, 3, 1>(0, 1, 0))
                    * Eigen::AngleAxis(yaw, Eigen::Matrix<ElementType, 3, 1>(0, 0, 1));
        }

        //! Default destructor.
        ~Quaternion() = default;

        //! Reference to underlying Eigen data.
        [[nodiscard]] EigenType &data() { return int_quat; }

        //! Const reference to underlying Eigen data.
        [[nodiscard]] const EigenType &data() const { return int_quat; }

        //! Returns the real component of the quaternion.
        /*!
         *
         * @return the real component of the quaternion.
         */
        [[nodiscard]] ElementType w() const { return int_quat.w(); }

        //! Returns the first imaginary component of the quaternion.
        /*!
         *
         * @return the first imaginary component of the quaternion.
         */
        [[nodiscard]] ElementType x() const { return int_quat.x(); }

        //! Returns the second imaginary component of the quaternion.
        /*!
         *
         * @return the second imaginary component of the quaternion.
         */
        [[nodiscard]] ElementType y() const { return int_quat.y(); }

        //! Returns the third imaginary component of the quaternion.
        /*!
         *
         * @return the third imaginary component of the quaternion.
         */
        [[nodiscard]] ElementType z() const { return int_quat.z(); }

        //! Returns the real component of the quaternion.
        /*!
         *
         * @return the real component of the quaternion.
         */
        [[nodiscard]] ElementType scalar() const { return int_quat.w(); }

        //! Returns imaginary components of the quaternion as a vector.
        /*!
         *
         * @return imaginary components of the quaternion as a vector.
         */
        [[nodiscard]] VectorType vector() const { return VectorType(int_quat.x(), int_quat.y(), int_quat.z()); }

        //! Decomposes unit quaternion into roll-pitch-yaw angles.
        /*!
         * Does not necessarily return RPY used for construction of the quaternion, other valid combinations representing the same rotation are possible as well.
         * @param roll rotation around \a x axis return parameter.
         * @param pitch rotation around \a y axis return parameter.
         * @param yaw rotation around \a z axis return parameter.
         */
        void rpy(ElementType &roll, ElementType &pitch, ElementType &yaw)
        {
            auto v = int_quat.toRotationMatrix().eulerAngles(0, 1, 2);
            roll = v[0];
            pitch = v[1];
            yaw = v[2];
        }

        //! Set the real component of the quaternion.
        /*!
         *
         * @param w new value of the real component.
         */
        void setW(ElementType w) { int_quat.w() = w; }

        //! Set the first imaginary component of the quaternion.
        /*!
         *
         * @param x new value of the first imaginary component.
         */
        void setX(ElementType x) { int_quat.x() = x; }

        //! Set the second imaginary component of the quaternion.
        /*!
         *
         * @param y new value of the second imaginary component.
         */
        void setY(ElementType y) { int_quat.y() = y; }

        //! Set the third imaginary component of the quaternion.
        /*!
         *
         * @param z new value of the third imaginary component.
         */
        void setZ(ElementType z) { int_quat.z() = z; }

        //! Set the real component of the quaternion.
        /*!
         *
         * @param scalar new value of the real component.
         */
        void setScalar(ElementType scalar) { int_quat.w() = scalar; }

        //! Set the imaginary components using a vector.
        /*!
         *
         * @param v new values of imaginary components \a x, \a y and \a z.
         */
        void setVector(const VectorType &v)
        {
            int_quat.x() = v.x();
            int_quat.y() = v.y();
            int_quat.z() = v.z();
        }

        //! Spherical linear interpolation between *this and \p q.
        /*!
         * Uniform interpolation of the transition between two rotations defined by *his and \p q. Requires both quaternions to be unit.
         * @param q unit rotation quaternion.
         * @param scale interpolation parameter in the range [0; 1].
         * @return interpolated quaternion.
         */
        Quaternion slerp(const Quaternion& q, ElementType scale)
        {
            return Quaternion(int_quat.slerp(scale, q.int_quat));
        }

        //! Assignment operator.
        Quaternion &operator=(const Quaternion q)
        {
            int_quat = q.int_quat;
            return *this;
        }

        //! Assigns the quaternion with the EigenType variable.
        Quaternion &operator=(const EigenType &eq)
        {
            int_quat = eq;
            return *this;
        }

        //! Addition of two quaternions.
        /*!
         *
         * @param q quaternion to be added to *this.
         * @return new quaternion representing sum of *this and \p q.
         */
        Quaternion operator+(const Quaternion &q) const
        {
            Quaternion ret;
            ret.int_quat.coeffs() = int_quat.coeffs() + q.int_quat.coeffs();
            return ret;
        }

        //! In-place addition.
        /*!
         * Adds \p q to *this.
         * @param q quaternion to be added.
         * @return reference to *this.
         */
        Quaternion &operator+=(Quaternion &q)
        {
            int_quat.coeffs() += q.int_quat.coeffs();
            return *this;
        }

        //! Subtraction of two quaternions.
        /*!
         *
         * @param q quaternion to be subtracted from *this.
         * @return new quaternion representing subtraction of \p q from *this.
         */
        Quaternion operator-(const Quaternion &q) const
        {
            Quaternion ret;
            ret.int_quat.coeffs() = int_quat.coeffs() - q.int_quat.coeffs();
            return ret;
        }

        //! In-place subtraction.
        /*!
         * Subtracts \p q from *this.
         * @param q quaternion to be subtracted.
         * @return reference to *this.
         */
        Quaternion &operator-=(Quaternion &q)
        {
            int_quat.coeffs() -= q.int_quat.coeffs();
            return *this;
        }

        //! Unary minus for coefficient negation.
        /*!
         *
         * @return new quaternion constructed with negated coefficients of *this.
         */
        Quaternion operator-() const
        {
            Quaternion ret;
            ret.int_quat.coeffs() = -int_quat.coeffs();
            return ret;
        }

        //! Quaternion multiplication.
        /*!
         *
         * @param q quaternion to be multiplied with *this.
         * @return new quaternion resulting from multiplication.
         */
        Quaternion operator*(const Quaternion &q) const
        {
            return Quaternion(int_quat * q.int_quat);
        }

        //! In-place quaternion multiplication.
        /*!
         *
         * @param q quaternion to be multiplied with *this.
         * @return reference to *this.
         */
        Quaternion &operator*=(const Quaternion &q)
        {
            int_quat *= q.int_quat;
            return *this;
        }

        //! Scalar multiplication.
        /*!
         *
         * @param s the multiplier.
         * @return new quaternion representing \p s multiple of *this.
         */
        Quaternion operator*(ElementType s) const
        {
            Quaternion ret;
            ret.int_quat.coeffs() = int_quat.coeffs() * s;
            return ret;
        }

        //! In-place multiplication.
        /*!
         *
         * @param s the multiplier.
         * @return reference to *this.
         */
        Quaternion &operator*=(ElementType s)
        {
            int_quat.coeffs() *= s;
            return *this;
        }

        //! Division by a scalar.
        /*!
         *
         * @param s the divisor.
         * @return new quaternion representing *this divided by \p s.
         */
        Quaternion operator/(ElementType s) const
        {
            Quaternion ret;
            ret.int_quat.coeffs() = int_quat.coeffs() / s;
            return ret;
        }

        //! In-place scalar division.
        /*!
         *
         * @param s the divisor.
         * @return reference to *this.
         */
        Quaternion &operator/=(ElementType s)
        {
            int_quat.coeffs() /= s;
            return *this;
        }

        //! Norm of the quaternion.
        /*!
         *
         * @return the norm.
         */
        [[nodiscard]] ElementType norm() const { return int_quat.norm(); }

        //! Squared norm of the quaternion.
        /*!
         *
         * @return the squared norm.
         */
        [[nodiscard]] ElementType normSquared() const { return int_quat.dot(int_quat); }

        //! Normalizes the quaternion to a unit length.
        void normalize() { int_quat.normalize(); }

        //! Return normalized copy of *this.
        /*!
         *
         * @return normalized copy of *this.
         */
        [[nodiscard]] Quaternion normalized() const { return Quaternion(int_quat.normalized()); }

        //! Inverts of the quaternion.
        void invert() { int_quat = int_quat.inverse(); }

        //! Return inverted copy of *this.
        /*!
         *
         * @return inverted copy of *this.
         */
        [[nodiscard]] Quaternion inverted() const { return Quaternion(int_quat.inverse()); }

        //! Conjugates the quaternion.
        void conjugate() { int_quat = int_quat.conjugate(); }

        //! Returns conjugates copy of *this.
        /*!
         *
         * @return conjugated copy of *this.
         */
        [[nodiscard]] Quaternion conjugated() const { return Quaternion(int_quat.conjugate()); }

        //! Converts a unit quaternion to a 3 by 3 rotation matrix.
        /*!
         *
         * @return the rotation matrix.
         */
        [[nodiscard]] Matrix<3, 3, ElementType> rotMat() const { return Matrix<3, 3, ElementType>(int_quat.toRotationMatrix()); }

        //! Distance function for quaternions.
        /*!
         * Computer a differece of \p q1 and \p q2 and returns its norm.
         * @param q1 first operand.
         * @param q2 second operand.
         * @return the distance.
         */
        static ElementType distance(const Quaternion &q1, const Quaternion &q2) { return (q2 - q1).norm(); }

        //! Squared distance function for quaternions.
        /*!
         * Computer a differece of \p q1 and \p q2 and returns its squared norm.
         * @param q1 first operand.
         * @param q2 second operand.
         * @return the squared distance.
         */
        static ElementType distanceSquared(const Quaternion &q1, const Quaternion &q2)
        {
            return (q2 - q1).normSquared();
        }

        //! Dot product - cosine of half the angle between two unit quaternions.
        /*!
         *
         * @param q1 first operand.
         * @param q2 second operand.
         * @return the dot product.
         */
        static ElementType dotProduct(const Quaternion &q1, const Quaternion &q2)
        {
            return q1.int_quat.dot(q2.int_quat);
        }

        //! Return unit quaternion representing rotation by zero angle - identity.
        static Quaternion identity()
        {
            return Quaternion(1.0, 0.0, 0.0, 0.0);
        }

        //! Return a new quaternion with all elements initialized by the user-supplied random generator.
        /*!
         * Randomness is fully specified by the callable object \p el_rnd_gen, in fact it does not have to be random at all.
         * @tparam ElRndSrc type of the random generating object.
         * @param el_rnd_gen callable object returning ElementType on invocation.
         * @return a new quaternion initialized by \p el_rnd_gen.
         */
        template<class ElRndSrc>
        static Quaternion random(const ElRndSrc &el_rnd_gen)
        {
            return Quaternion(el_rnd_gen(), el_rnd_gen(), el_rnd_gen(), el_rnd_gen());
        }

        //! Return a new random angle-axis constructed quaternion.
        /*!
         * Randomness is fully specified by the callable objects \p el_rnd_gen and \p ang_rnd_gen, in fact it does not have to be random at all.
         * @tparam AngRndSrc type of random angle generator.
         * @tparam ElRndSrc type of random axis element generator.
         * @param ang_rnd_gen angle generator.
         * @param el_rnd_gen axis element generator.
         * @return a new quaternion initialized by \p ang_rnd_gen and \p el_rnd_gen.
         */
        template<class AngRndSrc, class ElRndSrc>
        static Quaternion random(const AngRndSrc &ang_rnd_gen, const ElRndSrc &el_rnd_gen)
        {
            return Quaternion(ang_rnd_gen(), VectorType(el_rnd_gen(), el_rnd_gen(), el_rnd_gen()));
        }

    private:
        EigenType int_quat;
    };

    //! Multiplication by a scalar multiplier from the left.
    /*!
     *
     * @tparam E ElementType oof the quaternion and the multiplier.
     * @param s the multiplier.
     * @param q the quaternion.
     * @return new quaternion representing \p q multiplied by \p s.
     */
    template<typename E>
    Quaternion<E> operator*(E s, const Quaternion<E> &q)
    {
        Quaternion<E> ret = q * s;
        return ret;
    }
}

#endif // ROBOTICTEMPLATELIBRARY_QUATERNION_H
