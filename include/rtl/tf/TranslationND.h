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

#ifndef ROBOTICTEMPLATELIBRARY_TRANSLATIONND_H
#define ROBOTICTEMPLATELIBRARY_TRANSLATIONND_H

#include "rtl/core/VectorND.h"


namespace rtl
{
    template<int, typename>
    class RotationND;

    template<int, typename>
    class RigidTfND;
    //! Base template for N-dimensional translations.
    /*!
     * Non-constructable base for TranslationND<> template. Implements common functionality available in all dimensions.
     * @tparam dimensions dimensionality of the translation.
     * @tparam Element base type of vector elements.
     * @tparam ChildTemplate template inheriting this one (CRTP).
     */
    template<int dimensions, typename Element, template<int, typename> class ChildTemplate>
    class TranslationND_common
    {
        static_assert(dimensions > 0, "TranslationND must have at least one dimension");
    private:
        typedef ChildTemplate<dimensions, Element> ChildType;

    public:
        typedef Element ElementType;        //!< base type for vector elements.
        typedef Element DistanceType;       //!< return type of the distance function.
        typedef VectorND<dimensions, Element> VectorType;   //!< type of vector representing the translation.

        //! Default destructor.
        ~TranslationND_common() = default;

        //! Translation vector of the transformation.
        /*!
         *
         * @return the translation vector.
         */
        const VectorType& trVec() const
        {
            return int_translation;
        }

        //! Sets new translation vector.
        /*!
         *
         * @param translation the new translation vector.
         */
        void setTrVec(const VectorType &translation)
        {
            int_translation = translation;
        }

        //! In-place inversion of the translation transformation.
        /*!
         * Direction of the internal vector is reversed.
         */
        void invert()
        {
            int_translation = -int_translation;
        }

        //! Returns inverted variant of *this.
        /*!
         *
         * @return the inverted variant of *this.
         */
        ChildType inverted() const
        {
            return ChildType(-int_translation);
        }

        //! Casting to different ElementType.
        template<typename NewElement>
        ChildTemplate<dimensions, NewElement> cast() const
        {
            return ChildTemplate<dimensions, NewElement>(int_translation.template cast<NewElement>());
        }

        //! Functor call invoking the transformed(*this) method of given argument.
        /*!
         * \p T is required to implement T::transformed(const TranslationND<dimensions, ElementType>&) method.
         * @tparam T type of the argument to be translated.
         * @param t the argument to be translated.
         * @return the translated variant of the argument.
         */
        template<class T>
        decltype(std::declval<T>().transformed(ChildType())) operator()(const T &t) const
        {
            return t.transformed(childThis());
        }

        //! Returns translation performing first translation by *this and than by \p tr.
        /*!
         * @param tr the translation to be added.
         * @return new combined translation.
         */
        ChildType transformed(const TranslationND<dimensions, Element> &tr) const
        {
            return ChildType(int_translation + tr.trVec());
        }

        //! In-place transforms *this to perform first translation by *this and than by \p tr.
        /*!
         *
         * @param tr the translation to be added.
         */
        void transform(const TranslationND<dimensions, Element> &tr)
        {
            int_translation += tr.trVec();
        }

        //! Returns rigid transformation performing first translation by *this and than rotation by \p rot.
        /*!
         * @param rot the rotation to be added.
         * @return new rigid transformation.
         */
        RigidTfND<dimensions, Element> transformed(const RotationND<dimensions, Element> &rot) const
        {
            return RigidTfND<dimensions, Element>(rot, ChildType(rot.rotMat() * int_translation));
        }

        //! Returns rigid transformation performing first translation by *this and than rigid transformation by \p tf.
        /*!
         * @param tf the transformation to be added.
         * @return new rigid transformation.
         */
        RigidTfND<dimensions, Element> transformed(const RigidTfND<dimensions, Element> &tf) const
        {
            return RigidTfND<dimensions, Element>(tf.rot(), ChildType(tf.rotMat() * int_translation + tf.trVec()));
        }

        //! Distance function for translations.
        /*!
         * Corresponds to distance between internal translation vectors of \p tr1 and \p tr2.
         * @param tr1 first translation.
         * @param tr2 second translation.
         * @return distance between internal vectors.
         */
        static DistanceType distance(const TranslationND_common &tr1, const TranslationND_common &tr2)
        {
            return VectorType::distance(tr1.int_translation, tr2.int_translation);
        }

        //! Squared-distance function for translations.
        /*!
         * Corresponds to squared distance between internal translation vectors of \p tr1 and \p tr2.
         * @param tr1 first translation.
         * @param tr2 second translation.
         * @return squared distance between internal vectors.
         */
        static DistanceType distanceSquared(const TranslationND_common &tr1, const TranslationND_common &tr2)
        {
            return VectorType::distanceSquared(tr1.int_translation, tr2.int_translation);
        }

        //! Return a new translation, which, when applied, leaves the transformed object as is.
        /*!
         *
         * @return a new translation initialized to leave the transformed object unchanged.
         */
        static ChildType identity()
        {
            return ChildType(VectorType::zeros());
        }

        //! Return a new translation with all elements initialized by the user-supplied random generator.
        /*!
         * Randomness is fully specified by the callable object \p el_rnd_gen, in fact it does not have to be random at all.
         * @tparam ElRndSrc type of the random generating object.
         * @param el_rnd_gen callable object returning ElementType on invocation.
         * @return a new translation initialized by \p el_rnd_gen.
         */
        template<class ElRndSrc>
        static ChildType random(const ElRndSrc &el_rnd_gen)
        {
            return ChildType(VectorType::random(el_rnd_gen));
        }

    protected:
        TranslationND_common()= default;

        template<typename ...T, typename std::enable_if<sizeof...(T) == dimensions, int>::type = 0>
        explicit TranslationND_common(T ...args) : int_translation(args...) {}

        TranslationND_common(const TranslationND_common<dimensions, Element, ChildTemplate> &tr)
        {
            int_translation = tr.int_translation;
        }

        explicit TranslationND_common(const VectorType &vec)
        {
            int_translation = vec;
        }

        explicit TranslationND_common(const ChildType &tr)
        {
            int_translation = tr.int_translation;
        }

        operator ChildType() const
        {
            return ChildType(int_translation);
        }

        VectorType int_translation;

    private:
        TranslationND_common<dimensions, Element, ChildTemplate> &operator=(const TranslationND_common<dimensions, Element, ChildTemplate> &tr)
        {
            int_translation = tr.int_translation;
            return *this;
        }

        ChildType &childThis() { return static_cast<ChildType &>(*this); }
        const ChildType &childThis() const { return static_cast<ChildType const &>(*this); }
    };

    //! N-dimensional translation template.
    /*!
     * General implementation of translation transformations for any dimension. For dimensions of special interest (2D and 3D) there are specializations with additional functionality.
     * @tparam dimensions dimensionality of the translation.
     * @tparam Element type of the translation's elements.
     */
    template<int dimensions, typename Element>
    class TranslationND : public TranslationND_common<dimensions, Element, TranslationND>
    {
    public:
        typedef typename TranslationND_common<dimensions, Element, TranslationND>::ElementType ElementType;         //!< base type for vector elements.
        typedef typename TranslationND_common<dimensions, Element, TranslationND>::VectorType VectorType;           //!< type of vector representing the translation.

        //! Default constructor, the translation is uninitialized.
        TranslationND() = default;

        //! Copy constructor.
        TranslationND(const TranslationND<dimensions, Element> &tr) : TranslationND_common<dimensions, Element, TranslationND>(tr) {}

        //! Element-wise construction - number of arguments must correspond to translation's dimensionality.
        template<typename ...T, typename std::enable_if<sizeof...(T) == dimensions, int>::type = 0>
        explicit TranslationND(T ...args) : TranslationND_common<dimensions, Element, TranslationND>(args...) {}

        //! From vector construction.
        explicit TranslationND(const VectorType &vec) : TranslationND_common<dimensions, Element, TranslationND>(vec) {}

        //! Assignment operator.
        TranslationND<dimensions, Element> &operator=(const TranslationND<dimensions, Element> &tr)
        {
            this->int_translation = tr.int_translation;
            return *this;
        }
    };

    //! Two dimensional specialization of TranslationND template.
    /*!
     * Adds 2D-exclusive functions to the general implementation of an N-dimensional translation.
     * @tparam Element base type for data elements.
     */
    template<typename Element>
    class TranslationND<2, Element> : public TranslationND_common<2, Element, TranslationND>
    {
    public:
        typedef typename TranslationND_common<2, Element, TranslationND>::ElementType ElementType;         //!< base type for vector elements.
        typedef typename TranslationND_common<2, Element, TranslationND>::VectorType VectorType; //!< type of vector representing the translation.

        //! Default constructor, the translation is uninitialized.
        TranslationND() = default;

        //! Copy constructor.
        TranslationND(const TranslationND<2, Element> &tr) : TranslationND_common<2, Element, TranslationND>(tr) {}

        //! Element-wise construction.
        TranslationND(ElementType x, ElementType y) : TranslationND_common<2, Element, TranslationND>(x, y) {}

        //! From vector construction.
        explicit TranslationND(const VectorType &vec) : TranslationND_common<2, Element, TranslationND>(vec) {}

        //! Assignment operator.
        TranslationND<2, Element> &operator=(const TranslationND<2, Element> &tr)
        {
            this->int_translation = tr.int_translation;
            return *this;
        }

        //! \a x element of the translation vector of the transformation.
        /*!
         *
         * @return the \a x element of the translation vector.
         */
        ElementType trVecX() const
        {
            return this->int_translation.x();
        }

        //! \a y element of the translation vector of the transformation.
        /*!
         *
         * @return the \a y element of the translation vector.
         */
        ElementType trVecY() const
        {
            return this->int_translation.y();
        }

        //! Sets \a x element of the translation vector.
        /*!
         *
         * @param el new value of the \a x element of the translation vector.
         */
        void setTrVecX(ElementType el)
        {
            this->int_translation.setX(el);
        }

        //! Sets \a y element of the translation vector.
        /*!
         *
         * @param el new value of the \a y element of the translation vector.
         */
        void setTrVecY(ElementType el)
        {
            this->int_translation.setY(el);
        }
    };

    //! Three dimensional specialization of TranslationND template.
    /*!
     * Adds 3D-exclusive functions to the general implementation of an N-dimensional translation.
     * @tparam Element base type for data elements.
     */
    template<typename Element>
    class TranslationND<3, Element> : public TranslationND_common<3, Element, TranslationND>
    {
    public:
        typedef typename TranslationND_common<3, Element, TranslationND>::ElementType ElementType;         //!< base type for vector elements.
        typedef typename TranslationND_common<3, Element, TranslationND>::VectorType VectorType; //!< type of vector representing the translation.

        //! Default constructor, the translation is uninitialized.
        TranslationND() = default;

        //! Copy constructor.
        TranslationND(const TranslationND<3, Element> &tr) : TranslationND_common<3, Element, TranslationND>(tr) {}

        //! Element-wise construction.
        TranslationND(ElementType x, ElementType y, ElementType z) : TranslationND_common<3, Element, TranslationND>(x, y, z) {}

        //! From vector construction.
        explicit TranslationND(const VectorType &vec) : TranslationND_common<3, Element, TranslationND>(vec) {}

        //! Assignment operator.
        TranslationND<3, Element> &operator=(const TranslationND<3, Element> &tr)
        {
            this->int_translation = tr.int_translation;
            return *this;
        }

        //! \a x element of the translation vector of the transformation.
        /*!
         *
         * @return the \a x element of the translation vector.
         */
        ElementType trVecX() const
        {
            return this->int_translation.x();
        }

        //! \a y element of the translation vector of the transformation.
        /*!
         *
         * @return the \a y element of the translation vector.
         */
        ElementType trVecY() const
        {
            return this->int_translation.y();
        }

        //! \a z element of the translation vector of the transformation.
        /*!
         *
         * @return the \a z element of the translation vector.
         */
        ElementType trVecZ() const
        {
            return this->int_translation.z();
        }

        //! Sets \a x element of the translation vector.
        /*!
         *
         * @param el new value of the \a x element of the translation vector.
         */
        void setTrVecX(ElementType el)
        {
            this->int_translation.setX(el);
        }

        //! Sets \a y element of the translation vector.
        /*!
         *
         * @param el new value of the \a y element of the translation vector.
         */
        void setTrVecY(ElementType el)
        {
            this->int_translation.setY(el);
        }

        //! Sets \a z element of the translation vector.
        /*!
         *
         * @param el new value of the \a z element of the translation vector.
         */
        void setTrVecZ(ElementType el)
        {
            this->int_translation.setZ(el);
        }
    };
}

#endif //ROBOTICTEMPLATELIBRARY_TRANSLATIONND_H
