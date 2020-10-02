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

#ifndef ROBOTICTEMPLATELIBRARY_ROTATIONND_H
#define ROBOTICTEMPLATELIBRARY_ROTATIONND_H

#include "rtl/core/VectorND.h"
#include "rtl/core/Matrix.h"
#include "rtl/core/Quaternion.h"

namespace rtl
{
    template<int, typename>
    class TranslationND;

    template<int, typename>
    class RigidTfND;

    //! Base template for N-dimensional rotations.
    /*!
     * Non-constructable base for RotationND<> template. Implements common functionality available in all dimensions.
     * @tparam dimensions dimensionality of the rotation.
     * @tparam Element base type of data elements.
     * @tparam ChildTemplate template inheriting this one (CRTP).
     */
    template<int dimensions, typename Element, template<int, typename> class ChildTemplate>
    class RotationND_common
    {
        static_assert(dimensions > 1, "RotationND must have at least two dimensions.");
    private:
        typedef ChildTemplate<dimensions, Element> ChildType;

    public:
        typedef Element ElementType;        //!< base type for data elements.
        typedef Element DistanceType;       //!< return type of the distance function.
        typedef VectorND<dimensions, Element> VectorType;   //!< type of the vectors compatible with given rotation.
        typedef Matrix<dimensions, dimensions, ElementType> MatrixType; //!< type of the rotation matrix.

        //! Default destructor.
        ~RotationND_common() = default;

        //! Rotation matrix of the transformation.
        /*!
         *
         * @return the rotation matrix.
         */
        const MatrixType & rotMat() const
        {
            return int_rot_mat;
        }

        //! Sets new rotation using two vectors.
        /*!
         *
         * @param v1 vector to be rotated.
         * @param v2 required direction of \p v1 after the rotation is applied.
         */
        void setRot(const VectorType &v1, const VectorType &v2)
        {
            VectorType v1_on = v1.normalized();
            VectorType v2_on = (v2 - v1_on.dot(v2) * v1_on).normalized();
            VectorType v2_n = v2.normalized();
            ElementType cos_a = VectorType::scalarProjectionOnUnit(v2_n, v1_on);
            ElementType sin_a = VectorType::scalarProjectionOnUnit(v2_n, v2_on);
            int_rot_mat = MatrixType::identity() + sin_a * (v2_on.outer(v1_on) - v1_on.outer(v2_on)) + (cos_a - 1) * (v1_on.outer(v1_on) + v2_on.outer(v2_on));
            childThis().rotMatUpdated();
        }

        //! In-place inversion of the rotation transformation.
        /*!
         * The internal matrix is transposed.
         */
        void invert()
        {
            int_rot_mat.transpose();
            childThis().rotMatUpdated();
        }

        //! Returns inverted variant of *this.
        /*!
         *
         * @return the inverted variant of *this.
         */
        ChildType inverted() const
        {
            ChildType ret;
            ret.int_rot_mat = int_rot_mat.transposed();
            ret.rotMatUpdated();
            return ret;
        }

        //! Casting to different ElementType.
        template<typename NewElement>
        ChildTemplate<dimensions, NewElement> cast() const
        {
            ChildTemplate<dimensions, NewElement> ret;
            ret.int_rot_mat = int_rot_mat.template cast<NewElement>();
            ret.rotMatUpdated();
            return ret;
        }

        //! Functor call invoking the transformed(*this) method of given argument.
        /*!
         * \p T is required to implement T::transformed(const RotationND<dimensions, ElementType>&) method.
         * @tparam T type of the argument to be rotated.
         * @param t the argument to be rotated.
         * @return the rotated variant of the argument.
         */
        template<class T>
        auto operator()(const T &t) const
        {
            return t.transformed(childThis());
        }

        //! Returns rigid transformation performing first rotation by *this and than translation by \p tr.
        /*!
         * @param tr the translation to be added.
         * @return new rigid transformation.
         */
        RigidTfND<dimensions, Element> transformed(const TranslationND<dimensions, Element> &tr) const
        {
            return RigidTfND<dimensions, Element>(childThis(), tr);
        }

        //! Returns rotation performing first rotation by *this and than by \p rot.
        /*!
         * @param rot the rotation to be added.
         * @return new combined rotation.
         */
        ChildType transformed(const RotationND<dimensions, Element> &rot) const
        {
            ChildType ret(*this);
            ret.transform(rot);
            return ret;
        }

        //! In-place augments *this to perform first rotation by *this and than by \p rot.
        /*!
         *
         * @param rot the rotation to be added.
         */
        void transform(const RotationND<dimensions, Element> &rot)
        {
           int_rot_mat = rot.rotMat() * int_rot_mat;
           childThis().rotMatUpdated();
        }

        //! Returns rigid transformation performing first rotation by *this and than transformation by \p tr.
        /*!
         * @param tf the transformation to be added.
         * @return new rigid transformation.
         */
        RigidTfND<dimensions, Element> transformed(const RigidTfND<dimensions, Element> &tf) const
        {
            return RigidTfND<dimensions, Element>(tf.rot()(childThis()), tf.tr());
        }

        //! Return a new rotation, which, when applied, leaves the transformed object as is.
        /*!
         *
         * @return a new rotation initialized to leave the transformed object unchanged.
         */
        static ChildType identity()
        {
            ChildType ret;
            ret.int_rot_mat = MatrixType::identity();
            ret.rotMatUpdated();
            return ret;
        }

        //! Return a new rotation initialized by two vectors generated using the user-supplied random generator.
        /*!
         * Randomness is fully specified by the callable object \p el_rnd_gen, in fact it does not have to be random at all.
         * @tparam ElRndSrc type of the random generating object.
         * @param el_rnd_gen callable object returning ElementType on invocation.
         * @return a new rotation initialized by \p el_rnd_gen.
         */
        template<class ElRndSrc>
        static ChildType random(const ElRndSrc &el_rnd_gen)
        {
            return ChildType(VectorType::random(el_rnd_gen), VectorType::random(el_rnd_gen));
        }

        //! Dimensionality of the rotation.
        static constexpr int dimensionality() { return dimensions; }

    protected:
        RotationND_common()= default;

        RotationND_common(const RotationND_common<dimensions, Element, ChildTemplate> &tr)
        {
            int_rot_mat = tr.int_rot_mat;
        }

        explicit RotationND_common(const ChildType &tr)
        {
            int_rot_mat = tr.int_rot_mat;
        }

        RotationND_common(const VectorType &v1, const VectorType &v2)
        {
            childThis().setRot(v1, v2);
        }

        operator ChildType() const
        {
            ChildType ret;
            ret.int_rot_mat = int_rot_mat;
            ret.rotMatUpdated();
            return ret;
        }

        MatrixType int_rot_mat;

    private:
        RotationND_common<dimensions, Element, ChildTemplate> &operator=(const RotationND_common<dimensions, Element, ChildTemplate> &rot)
        {
            int_rot_mat = rot.int_rot_mat;
            return *this;
        }

        void rotMatUpdated() {}
        ChildType &childThis() { return static_cast<ChildType &>(*this); }
        const ChildType &childThis() const { return static_cast<ChildType const &>(*this); }
    };

    //! N-dimensional rotation template.
    /*!
     * General implementation of rotation transformations for any dimension. For dimensions of special interest (2D and 3D) there are specializations with additional functionality.
     * Allows direct construction of basic rotation using two vectors specifying the plane of rotation and the angle. In higher dimensions (\p dimensions > 3), there exist rotations,
     * which cannot be represented this way and are the result of composition of basic rotations. These compound rotation can be formed by application of one basic rotation
     * on another until the desired composition is obtained.
     * @tparam dimensions dimensionality of the rotation.
     * @tparam Element type of the data elements.
     */
    template<int dimensions, typename Element>
    class RotationND : public RotationND_common<dimensions, Element, RotationND>
    {
    public:
        typedef typename RotationND_common<dimensions, Element, RotationND>::ElementType ElementType;         //!< base type for data elements.
        typedef typename RotationND_common<dimensions, Element, RotationND>::VectorType VectorType;           //!< type of the vectors compatible with given rotation.

        //! Default constructor, the translation is uninitialized.
        RotationND() = default;

        //! Copy constructor.
        RotationND(const RotationND<dimensions, Element> &rot) : RotationND_common<dimensions, Element, RotationND>(rot) {}

        //! Two vector construction.
        RotationND(const VectorType &v1, const VectorType &v2) : RotationND_common<dimensions, Element, RotationND>(v1, v2) {}

        //! Assignment operator.
        RotationND<dimensions, Element> &operator=(const RotationND<dimensions, Element> &rot)
        {
            this->int_rot_mat = rot.int_rot_mat;
            return *this;
        }
    };

    //! Two dimensional specialization of RotationND template.
    /*!
     * Adds 2D-exclusive functions to the general implementation of an N-dimensional rotation.
     * @tparam Element base type for data elements.
     */
    template<typename Element>
    class RotationND<2, Element> : public RotationND_common<2, Element, RotationND>
    {
    public:
        typedef typename RotationND_common<2, Element, RotationND>::ElementType ElementType;  //!< base type for data elements.
        typedef typename RotationND_common<2, Element, RotationND>::VectorType VectorType;    //!< type of the vectors compatible with given rotation.
        typedef typename RotationND_common<2, Element, RotationND>::MatrixType MatrixType;    //!< type of the rotation matrix.

        //! Default constructor, the rotation is uninitialized.
        RotationND() = default;

        //! Copy constructor.
        RotationND(const RotationND<2, Element> &rot) : RotationND_common<2, Element, RotationND>(rot) {}

        //! Two vector construction.
        RotationND(const VectorType &v1, const VectorType &v2) : RotationND_common<2, Element, RotationND>(v1, v2) {}

        //! Angle constructor.
        /*!
         *
         * @param angle rotation angle in the counter-clockwise direction in radians.
         */
        explicit RotationND(ElementType angle)
        {
            setAngle(angle);
        }

        //! Sets new rotation using two vectors.
        /*!
         *
         * @param v1 vector to be rotated.
         * @param v2 required direction of \p v1 after the rotation is applied.
         */
        void setRot(const VectorType &v1, const VectorType &v2)
        {
            VectorType v1_on = v1.normalized();
            VectorType v2_on(-v1_on.y(), v1_on.x());
            VectorType v2_n = v2.normalized();
            ElementType cos_a = v1_on.dot(v2_n);
            ElementType sin_a = v2_on.dot(v2_n);
            this->int_rot_mat(0, 0) = cos_a;
            this->int_rot_mat(0, 1) = -sin_a;
            this->int_rot_mat(1, 0) = sin_a;
            this->int_rot_mat(1, 1) = cos_a;
        }

        //! Assignment operator.
        RotationND<2, Element> &operator=(const RotationND<2, Element> &rot)
        {
            this->int_rot_mat = rot.int_rot_mat;
            return *this;
        }

        //! Cosine of the angle of rotation.
        /*!
         *
         * @return cosine of the rotation angle.
         */
        Element rotCos() const
        {
            return this->int_rot_mat(0, 0);
        }

        //! Sine of the angle of rotation.
        /*!
         *
         * @return Sine of the rotation angle.
         */
        Element rotSin() const
        {
            return this->int_rot_mat(1, 0);
        }

        //! Rotation angle in counter-clockwise direction.
        /*!
         *
         * @return rotation angle in radians.
         */
        Element rotAngle() const
        {
            return std::atan2(rotSin(), rotCos());
        }

        //! Recalculate the rotation for given angle.
        /*!
         *
         * @param angle rotation angle in the counter-clockwise direction in radians.
         */
        void setAngle(Element angle)
        {
            this->int_rot_mat(0, 0) = this->int_rot_mat(1, 1) = std::cos(angle);
            this->int_rot_mat(1, 0) = std::sin(angle);
            this->int_rot_mat(0, 1) = -this->int_rot_mat(1, 0);
        }
    };

    //! Three dimensional specialization of RotationND template.
    /*!
     * Adds 3D-exclusive functions to the general implementation of an N-dimensional rotation.
     * @tparam Element base type for data elements.
     */
    template<typename Element>
    class RotationND<3, Element> : public RotationND_common<3, Element, RotationND>
    {
    public:
        typedef typename RotationND_common<3, Element, RotationND>::ElementType ElementType;         //!< base type for data elements.
        typedef typename RotationND_common<3, Element, RotationND>::VectorType VectorType; //!< type of the vectors compatible with given rotation.
        typedef typename RotationND_common<3, Element, RotationND>::MatrixType MatrixType; //!< type of the rotation matrix.

        //! Default constructor, the translation is uninitialized.
        RotationND() = default;

        //! Copy constructor.
        RotationND(const RotationND<3, Element> &rot) : RotationND_common<3, Element, RotationND>(rot), int_aa(rot.int_aa) {}

        //! Two vector construction.
        RotationND(const VectorType &v1, const VectorType &v2)
        {
            setRot(v1, v2);
        }

        //! Quaternion construction.
        explicit RotationND(const Quaternion <ElementType> &quat) : int_aa(quat.data())
        {
            this->int_rot_mat = quat.rotMat();
        }

        //! Angle axis constructor.
        RotationND(ElementType angle, const VectorType &axis) : int_aa(angle, axis.normalized().data())
        {
            this->int_rot_mat = int_aa.toRotationMatrix();
        }

        //! Roll-pitch-yaw constructor.
        RotationND(ElementType roll, ElementType pitch, ElementType yaw) : RotationND(Quaternion<ElementType>(roll, pitch, yaw)) {}

        //! Assignment operator.
        RotationND<3, Element> &operator=(const RotationND<3, Element> &rot)
        {
            this->int_rot_mat = rot.int_rot_mat;
            int_aa = rot.int_aa;
            return *this;
        }

        //! Sets new rotation using two vectors.
        /*!
         *
         * @param v1 vector to be rotated.
         * @param v2 required direction of \p v1 after the rotation is applied.
         */
        void setRot(const VectorType &v1, const VectorType &v2)
        {
            int_aa = Quaternion<ElementType>(v1, v2).data();
            this->int_rot_mat = int_aa.toRotationMatrix();
        }

        //! Cosine of the angle of rotation.
        /*!
         *
         * @return cosine of the rotation angle.
         */
        Element rotCos() const
        {
            return std::cos(int_aa.angle());
        }

        //! Sine of the angle of rotation.
        /*!
         *
         * @return Sine of the rotation angle.
         */
        Element rotSin() const
        {
            return std::sin(int_aa.angle());
        }

        //! Rotation angle.
        /*!
         *
         * @return rotation angle in radians.
         */
        [[nodiscard]] Element rotAngle() const
        {
            return int_aa.angle();
        }

        //! Rotation axis.
        /*!
         *
         * @return unit length axis of rotation.
         */
        [[nodiscard]] VectorND<3, Element> rotAxis() const
        {
            return VectorND<3, Element>(int_aa.axis());
        }

        //! Equivalent quaternion.
        /*!
         *
         * @return  quaternion representing *this rotation.
         */
        [[nodiscard]] Quaternion<Element> rotQuaternion() const
        {
            Eigen::Quaternion<Element> q;
            q = int_aa;
            return Quaternion(q.w(), q.x(), q.y(), q.z());
        }

        //! Equivalent roll-pitch-yaw angles.
        /*!
         * Does not necessarily return RPY used for construction of the transformation, other valid combinations representing the same rotation are possible as well.
         * @param roll roll-angle return argument.
         * @param pitch pitch-angle return argument.
         * @param yaw yaw-angle return argument.
         */
        void rotRpy(ElementType &roll, ElementType &pitch, ElementType &yaw)
        {
            auto v = this->int_rot_mat.data().eulerAngles(0, 1, 2);
            roll = v[0];
            pitch = v[1];
            yaw = v[2];
        }

        //! Set rotation angle with axis unchanged.
        /*!
         *
         * @param angle new angle in radians.
         */
        void setAngle(ElementType angle)
        {
            int_aa.angle() = angle;
            this->int_rot_mat = int_aa.toRotationMatrix();
        }

        //! Set rotation axis with angle unchanged.
        /*!
         *
         * @param axis new axis.
         */
        void setAxis(const VectorND<3, Element> &axis)
        {
            int_aa.axis() = axis.normalized().data();
            this->int_rot_mat = int_aa.toRotationMatrix();
        }

        //! Recompute the rotation with a new angle and axis.
        /*!
         *
         * @param angle new angle in radians.
         * @param axis new axis.
         */
        void setAngleAxis(Element angle, const VectorND<3, Element> &axis)
        {
            int_aa.angle() = angle;
            int_aa.axis() = axis.normalized().data();
            this->int_rot_mat = int_aa.toRotationMatrix();
        }

        //! Return a new rotation object leaving the transformed object as is.
        /*!
         *
         * @return a new rotation initialized to leave the transformed object unchanged.
         */
        static RotationND<3, Element> identity()
        {
            RotationND<3, Element> ret;
            ret.int_rot_mat = MatrixType::identity();
            ret.int_aa.angle() = 0;
            ret.int_aa.axis() = VectorType::baseX().data();
            return ret;
        }

    private:
        friend RotationND_common<3, Element, RotationND>;

        void rotMatUpdated()
        {
            int_aa = Eigen::AngleAxis<Element>(this->int_rot_mat.data());
        }

        Eigen::AngleAxis<Element> int_aa;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_ROTATIONND_H
