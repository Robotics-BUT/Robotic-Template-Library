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

#ifndef ROBOTICTEMPLATELIBRARY_RIGIDTFND_H
#define ROBOTICTEMPLATELIBRARY_RIGIDTFND_H

#include "rtl/tf/TranslationND.h"
#include "rtl/tf/RotationND.h"


namespace rtl
{
    template<int, typename>
    class TranslationND;

    template<int, typename>
    class RotationND;

    //! Base template for N-dimensional rigid transformation.
    /*!
     * Non-constructable base for RigidTfND<> template. Implements common functionality available in all dimensions.
     * @tparam dimensions dimensionality of the transformation.
     * @tparam Element base type of vector elements.
     * @tparam ChildTemplate template inheriting this one (CRTP).
     */
    template<int dimensions, typename Element, template<int, typename> class ChildTemplate>
    class RigidTfND_common
    {
        static_assert(dimensions > 0, "TranslationND must have at least one dimension");
    private:
        typedef ChildTemplate<dimensions, Element> ChildType;

    public:
        typedef Element ElementType;        //!< base type for vector elements.
        typedef VectorND<dimensions, Element> VectorType;   //!< type of vector representing the translation.
        typedef TranslationND<dimensions, Element> TranslationType; //!< type of internal translation transformation.
        typedef RotationND<dimensions, Element> RotationType; //!< type of internal rotation transformation.
        typedef typename RotationType::MatrixType MatrixType; //!< type of the rotation matrix of the internal rotation transformation.

        //! Default destructor.
        ~RigidTfND_common() = default;

        //! Translation part of the rigid transformation.
        /*!
         *
         * @return the internal translation.
         */
        const TranslationType& tr() const
        {
            return int_translation;
        }

        //! Translation vector of the transformation.
        /*!
         *
         * @return the translation vector.
         */
        const VectorType& trVec() const
        {
            return int_translation.trVec();
        }

        //! Sets new translation vector.
        /*!
         *
         * @param translation the new translation vector.
         */
        void setTrVec(const VectorType &translation)
        {
            int_translation.setTrVec(translation);
        }

        //! Rotation part of the transformation.
        /*!
         *
         * @return the internal rotation.
         */
        const RotationType& rot() const
        {
            return int_rotation;
        }
        //! Rotation matrix of the transformation.
        /*!
         *
         * @return the rotation matrix.
         */
        const MatrixType & rotMat() const
        {
            return int_rotation.rotMat();
        }

        //! Sets new rotation using two vectors.
        /*!
         *
         * @param v1 vector to be rotated.
         * @param v2 required direction of \p v1 after the rotation is applied.
         */
        void setRot(const VectorType &v1, const VectorType &v2)
        {
            int_rotation.setRot(v1, v2);
        }

        //! In-place inversion of the translation transformation.
        void invert()
        {
            int_rotation.invert();
            int_translation.setTrVec(-(int_rotation.rotMat() * int_translation.trVec()));
        }

        //! Returns inverted variant of *this.
        /*!
         *
         * @return the inverted variant of *this.
         */
        ChildType inverted() const
        {
            ChildType ret(*this);
            ret.invert();
            return ret;
        }

        //! Casting to different ElementType.
        template<typename NewElement>
        ChildTemplate<dimensions, NewElement> cast() const
        {
            return ChildTemplate<dimensions, NewElement>(int_rotation.template cast<NewElement>(), int_translation.template cast<NewElement>());
        }

        //! Functor call invoking the transformed(*this) method of given argument.
        /*!
         * \p T is required to implement T::transformed(const RigidTfND<dimensions, ElementType>&) method.
         * @tparam T type of the argument to be translated.
         * @param t the argument to be translated.
         * @return the translated variant of the argument.
         */
        template<class T>
        auto operator()(const T &t) const
        {
            return t.transformed(childThis());
        }

        //! Returns rigid transformation performing first transformation by *this and than translation by \p tr.
        /*!
         * @param tr the translation to be added.
         * @return new rigid transformation.
         */
        ChildType transformed(const TranslationND<dimensions, Element> &tr) const
        {
            return ChildType(int_rotation, tr(int_translation));
        }

        //! In-place augments *this to perform first transformation by *this and than translation by \p tr.
        /*!
         *
         * @param tr the translation to be added.
         */
        void transform(const TranslationND<dimensions, Element> &tr)
        {
            int_translation.transform(tr);
        }

        //! Returns rigid transformation performing first transformation by *this and than rotation by \p rot.
        /*!
         * @param rot the rotation to be added.
         * @return new rigid transformation.
         */
        ChildType transformed(const RotationND<dimensions, Element> &rot) const
        {
            return ChildType(rot(int_rotation), TranslationType(rot(int_translation.trVec())));
        }

        //! In-place augments *this to perform first transformation by *this and than rotation by \p rot.
        /*!
         *
         * @param rot the rotation to be added.
         */
        void transform(const RotationND<dimensions, Element> &rot)
        {
            int_rotation.transform(rot);
            int_translation.setTrVec(rot(int_translation.trVec()));
        }

        //! Returns rigid transformation performing first transformation by *this and than by \p tf.
        /*!
         * @param tf the transformation to be added.
         * @return new combined rigid transformation.
         */
        ChildType transformed(const ChildType &tf) const
        {
            return ChildType(tf.rot()(int_rotation), TranslationType(tf.rot()(int_translation.trVec()) + tf.trVec()));
        }

        //! In-place augments *this to perform first transformation by *this and than by \p tf.
        /*!
         *
         * @param tf the transformation to be added.
         */
        void transform(const ChildType &tf)
        {
            int_rotation.transform(tf.rot());
            int_translation.setTrVec(tf.rot()(int_translation.trVec()) + tf.trVec());
        }

        //! Return a new transformation, which, when applied, leaves the transformed object as is.
        /*!
         *
         * @return a new translation initialized to leave the transformed object unchanged.
         */
        static ChildType identity()
        {
            return ChildType(RotationType::identity(), TranslationType::identity());
        }

        //! Return a new rigid transformation with both rotation and translation initialized by the user-supplied random generator.
        /*!
         * Randomness is fully specified by the callable object \p el_rnd_gen, in fact it does not have to be random at all.
         * @tparam ElRndSrc type of the random generating object.
         * @param el_rnd_gen callable object returning ElementType on invocation.
         * @return a new translation initialized by \p el_rnd_gen.
         */
        template<class ElRndSrc>
        static ChildType random(const ElRndSrc &el_rnd_gen)
        {
            return ChildType(RotationType::random(el_rnd_gen), TranslationType::random(el_rnd_gen));
        }

        //! Dimensionality of the rigid transformation.
        static constexpr int dimensionality() { return dimensions; }

    protected:
        RigidTfND_common()= default;

        RigidTfND_common(const RigidTfND_common<dimensions, Element, ChildTemplate> &tr)
        {
            int_translation = tr.int_translation;
            int_rotation = tr.int_rotation;
        }

        explicit RigidTfND_common(const TranslationType &tr)
        {
            int_translation = tr;
            int_rotation = RotationType::identity();
        }

        explicit RigidTfND_common(const RotationType &rot)
        {
            int_translation = TranslationType::identity();
            int_rotation = rot;
        }

        RigidTfND_common(const RotationType &rot, const TranslationType &tr)
        {
            int_translation = tr;
            int_rotation = rot;
        }

        RigidTfND_common(const VectorType &rot_from, const VectorType &rot_to, const VectorType tr) : int_rotation(rot_from, rot_to), int_translation(tr)
        {
        }

        explicit RigidTfND_common(const ChildType &tr)
        {
            int_translation = tr.int_translation;
            int_rotation = tr.int_rotation;
        }

        operator ChildType() const
        {
            return ChildType(childThis());
        }

        TranslationType int_translation;
        RotationType int_rotation;

    private:
        RigidTfND_common<dimensions, Element, ChildTemplate> &operator=(const RigidTfND_common<dimensions, Element, ChildTemplate> &tr)
        {
            int_translation = tr.int_translation;
            return *this;
        }

        ChildType &childThis() { return static_cast<ChildType &>(*this); }
        const ChildType &childThis() const { return static_cast<ChildType const &>(*this); }
    };

    //! N-dimensional rigid transformation template.
    /*!
     * General implementation of rigid transformation for any dimension. For dimensions of special interest (2D and 3D) there are specializations with additional functionality. Note
     * that this is "pure" rigid transformation, i.e. only rotation and translation is performed and reflection is not supported.
     * @tparam dimensions dimensionality of the translation.
     * @tparam Element type of the translation's elements.
     */
    template<int dimensions, typename Element>
    class RigidTfND : public RigidTfND_common<dimensions, Element, RigidTfND>
    {
    public:
        typedef typename RigidTfND_common<dimensions, Element, RigidTfND>::ElementType ElementType;         //!< base type for vector elements.
        typedef typename RigidTfND_common<dimensions, Element, RigidTfND>::VectorType VectorType;           //!< type of vector representing the translation.
        typedef typename RigidTfND_common<dimensions, Element, RigidTfND>::TranslationType TranslationType; //!< type of internal translation transformation.
        typedef typename RigidTfND_common<dimensions, Element, RigidTfND>::RotationType RotationType;       //!< type of internal rotation transformation.

        //! Default constructor, the transformation is uninitialized.
        RigidTfND() = default;

        //! Copy constructor.
        RigidTfND(const RigidTfND<dimensions, Element> &tr) : RigidTfND_common<dimensions, Element, RigidTfND>(tr) {}

        //! From translation constructor. Rotation is initialized to identity.
        explicit RigidTfND(const TranslationType &tr) : RigidTfND_common<dimensions, Element, RigidTfND>(tr) {}

        //! From rotation constructor. Translation is initialized to identity.
        explicit RigidTfND(const RotationType &rot) : RigidTfND_common<dimensions, Element, RigidTfND>(rot) {}

        //! Construction from stand-alone rotation and translation.
        RigidTfND(const RotationType &rot, const TranslationType &tr) : RigidTfND_common<dimensions, Element, RigidTfND>(rot, tr) {}

        //! Construction from vectors specifying the rotation and the translation.
        RigidTfND(const VectorType &rot_from, const VectorType &rot_to, const VectorType tr) : RigidTfND_common<dimensions, Element, RigidTfND>(rot_from, rot_to, tr) {}

        //! Assignment operator.
        RigidTfND<dimensions, Element> &operator=(const RigidTfND<dimensions, Element> &tr)
        {
            this->int_translation = tr.int_translation;
            this->int_rotation = tr.int_rotation;
            return *this;
        }
    };

    //! Two dimensional specialization of TranslationND template.
    /*!
     * Adds 2D-exclusive functions to the general implementation of an N-dimensional translation.
     * @tparam Element base type for data elements.
     */
    template<typename Element>
    class RigidTfND<2, Element> : public RigidTfND_common<2, Element, RigidTfND>
    {
    public:
        typedef typename RigidTfND_common<2, Element, RigidTfND>::ElementType ElementType;         //!< base type for vector elements.
        typedef typename RigidTfND_common<2, Element, RigidTfND>::VectorType VectorType;           //!< type of vector representing the translation.
        typedef typename RigidTfND_common<2, Element, RigidTfND>::TranslationType TranslationType; //!< type of internal translation transformation.
        typedef typename RigidTfND_common<2, Element, RigidTfND>::RotationType RotationType;       //!< type of internal rotation transformation.

        //! Default constructor, the transformation is uninitialized.
        RigidTfND() = default;

        //! Copy constructor.
        RigidTfND(const RigidTfND<2, Element> &tr) : RigidTfND_common<2, Element, RigidTfND>(tr) {}

        //! From translation constructor. Rotation is initialized to identity.
        explicit RigidTfND(const TranslationType &tr) : RigidTfND_common<2, Element, RigidTfND>(tr) {}

        //! From rotation constructor. Translation is initialized to identity.
        explicit RigidTfND(const RotationType &rot) : RigidTfND_common<2, Element, RigidTfND>(rot) {}

        //! Construction from stand-alone rotation and translation.
        RigidTfND(const RotationType &rot, const TranslationType &tr) : RigidTfND_common<2, Element, RigidTfND>(rot, tr) {}

        //! Construction from vectors specifying the rotation and the translation.
        RigidTfND(const VectorType &rot_from, const VectorType &rot_to, const VectorType tr) : RigidTfND_common<2, Element, RigidTfND>(rot_from, rot_to, tr) {}

        //! Construction from angle in radians and translation along x and y axes.
        RigidTfND(Element angle, Element tr_x, Element tr_y) : RigidTfND_common<2, Element, RigidTfND>(RotationType(angle), TranslationType(tr_x, tr_y)) {}

        //! Construction from angle in radians and a translation vector.
        RigidTfND(Element angle, const VectorType &tr) : RigidTfND_common<2, Element, RigidTfND>(RotationType(angle), TranslationType(tr)) {}

        //! Assignment operator.
        RigidTfND<2, Element> &operator=(const RigidTfND<2, Element> &tr)
        {
            this->int_translation = tr.int_translation;
            this->int_rotation = tr.int_rotation;
            return *this;
        }

        //! \a x element of the translation vector of the transformation.
        /*!
         *
         * @return the \a x element of the translation vector.
         */
        ElementType trVecX() const
        {
            return this->int_translation.trVecX();
        }

        //! \a y element of the translation vector of the transformation.
        /*!
         *
         * @return the \a y element of the translation vector.
         */
        ElementType trVecY() const
        {
            return this->int_translation.trVecY();
        }

        //! Sets \a x element of the translation vector.
        /*!
         *
         * @param el new value of the \a x element of the translation vector.
         */
        void setTrVecX(ElementType el)
        {
            this->int_translation.setTrVecX(el);
        }

        //! Sets \a y element of the translation vector.
        /*!
         *
         * @param el new value of the \a y element of the translation vector.
         */
        void setTrVecY(ElementType el)
        {
            this->int_translation.setTrVecY(el);
        }

        //! Cosine of the angle of rotation.
        /*!
         *
         * @return cosine of the rotation angle.
         */
        Element rotCos() const
        {
            return this->int_rotation.rotCos();
        }

        //! Sine of the angle of rotation.
        /*!
         *
         * @return Sine of the rotation angle.
         */
        Element rotSin() const
        {
            return this->int_rotation.rotSin();
        }

        //! Rotation angle in counter-clockwise direction.
        /*!
         *
         * @return rotation angle in radians.
         */
        Element rotAngle() const
        {
            return this->int_rotation.rotAngle();
        }

        //! Recalculate the rotation for given angle.
        /*!
         *
         * @param angle rotation angle in the counter-clockwise direction.
         */
        void setAngle(Element angle)
        {
            this->int_rotation.setAngle(angle);
        }
    };

    //! Three dimensional specialization of TranslationND template.
    /*!
     * Adds 3D-exclusive functions to the general implementation of an N-dimensional translation.
     * @tparam Element base type for data elements.
     */
    template<typename Element>
    class RigidTfND<3, Element> : public RigidTfND_common<3, Element, RigidTfND>
    {
    public:
        typedef typename RigidTfND_common<3, Element, RigidTfND>::ElementType ElementType;         //!< base type for vector elements.
        typedef typename RigidTfND_common<3, Element, RigidTfND>::VectorType VectorType;           //!< type of vector representing the translation.
        typedef typename RigidTfND_common<3, Element, RigidTfND>::TranslationType TranslationType; //!< type of internal translation transformation.
        typedef typename RigidTfND_common<3, Element, RigidTfND>::RotationType RotationType;       //!< type of internal rotation transformation.

        //! Default constructor, the transformation is uninitialized.
        RigidTfND() = default;

        //! Copy constructor.
        RigidTfND(const RigidTfND<3, Element> &tr) : RigidTfND_common<3, Element, RigidTfND>(tr) {}

        //! From translation constructor. Rotation is initialized to identity.
        explicit RigidTfND(const TranslationType &tr) : RigidTfND_common<3, Element, RigidTfND>(tr) {}

        //! From rotation constructor. Translation is initialized to identity.
        explicit RigidTfND(const RotationType &rot) : RigidTfND_common<3, Element, RigidTfND>(rot) {}

        //! Construction from stand-alone rotation and translation.
        RigidTfND(const RotationType &rot, const TranslationType &tr) : RigidTfND_common<3, Element, RigidTfND>(rot, tr) {}

        //! Construction from vectors specifying the rotation and the translations.
        RigidTfND(const VectorType &rot_from, const VectorType &rot_to, const VectorType tr) : RigidTfND_common<3, Element, RigidTfND>(rot_from, rot_to, tr) {}

        //! Construction from an angle-axis representation of the rotation and a translation vector.
        RigidTfND(Element angle, const VectorType &axis, const VectorType &tr) : RigidTfND_common<3, Element, RigidTfND>(RotationType(angle, axis), TranslationType(tr)) {}

        //! Construction from a rotation quaternion and a translation vector.
        RigidTfND(const Quaternion<Element> &quat, const VectorType &tr) : RigidTfND_common<3, Element, RigidTfND>(RotationType(quat), TranslationType(tr)) {}

        //! Construction from roll-pitch-yaw format of rotation and a translation vector.
        RigidTfND(ElementType roll, ElementType pitch, ElementType yaw, const VectorType &tr) : RigidTfND_common<3, Element, RigidTfND>(RotationType(roll, pitch, yaw), TranslationType(tr)) {}

        //! Assignment operator.
        RigidTfND<3, Element> &operator=(const RigidTfND<3, Element> &tr)
        {
            this->int_translation = tr.int_translation;
            this->int_rotation = tr.int_rotation;
            return *this;
        }

        //! \a x element of the translation vector of the transformation.
        /*!
         *
         * @return the \a x element of the translation vector.
         */
        ElementType trVecX() const
        {
            return this->int_translation.trVecX();
        }

        //! \a y element of the translation vector of the transformation.
        /*!
         *
         * @return the \a y element of the translation vector.
         */
        ElementType trVecY() const
        {
            return this->int_translation.trVecY();
        }

        //! \a z element of the translation vector of the transformation.
        /*!
         *
         * @return the \a z element of the translation vector.
         */
        ElementType trVecZ() const
        {
            return this->int_translation.trVecZ();
        }

        //! Sets \a x element of the translation vector.
        /*!
         *
         * @param el new value of the \a x element of the translation vector.
         */
        void setTrVecX(ElementType el)
        {
            this->int_translation.setTrVecX(el);
        }

        //! Sets \a y element of the translation vector.
        /*!
         *
         * @param el new value of the \a y element of the translation vector.
         */
        void setTrVecY(ElementType el)
        {
            this->int_translation.setTrVecY(el);
        }

        //! Sets \a z element of the translation vector.
        /*!
         *
         * @param el new value of the \a z element of the translation vector.
         */
        void setTrVecZ(ElementType el)
        {
            this->int_translation.setTrVecZ(el);
        }

        //! Cosine of the angle of rotation.
        /*!
         *
         * @return cosine of the rotation angle.
         */
        Element rotCos() const
        {
            return this->int_rotation.rotCos();
        }

        //! Sine of the angle of rotation.
        /*!
         *
         * @return Sine of the rotation angle.
         */
        Element rotSin() const
        {
            return this->int_rotation.rotSin();
        }

        //! Rotation angle.
        /*!
         *
         * @return rotation angle in radians.
         */
        [[nodiscard]] Element rotAngle() const
        {
            return this->int_rotation.rotAngle();
        }

        //! Rotation axis.
        /*!
         *
         * @return unit length axis of rotation.
         */
        [[nodiscard]] VectorType rotAxis() const
        {
            return this->int_rotation.rotAxis();
        }

        //! Equivalent quaternion.
        /*!
         *
         * @return  quaternion representing *this rotation.
         */
        [[nodiscard]] Quaternion<Element> rotQuaternion() const
        {
            return this->int_rotation.rotQuaternion();
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
            this->int_rotation.rotRpy(roll, pitch, yaw);
        }

        //! Set rotation angle with axis unchanged.
        /*!
         *
         * @param angle new angle in radians.
         */
        void setAngle(ElementType angle)
        {
            this->int_rotation.setAngle(angle);
        }

        //! Set rotation axis with angle unchanged.
        /*!
         *
         * @param axis new axis.
         */
        void setAxis(const VectorND<3, Element> &axis)
        {
            this->int_rotation.setAxis(axis);
        }

        //! Recompute the rotation with a new angle and axis.
        /*!
         *
         * @param angle new angle in radians.
         * @param axis new axis.
         */
        void setAngleAxis(Element angle, const VectorND<3, Element> &axis)
        {
            this->int_rotation.setAngleAxis(angle, axis);
        }

    };
}


#endif //ROBOTICTEMPLATELIBRARY_RIGIDTFND_H
