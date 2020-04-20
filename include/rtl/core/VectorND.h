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

#ifndef ROBOTICTEMPLATELIBRARY_VECTORND_H
#define ROBOTICTEMPLATELIBRARY_VECTORND_H

#include <cmath>
#include <type_traits>
#include <eigen3/Eigen/Dense>

namespace rtl
{
    template<int rows, int cols, typename Element>
    class Matrix;

    template<typename Element>
    class Transformation2D;

    template<typename Element>
    class Transformation3D;

    //! Base template for N-dimensional vectors.
    /*!
     * Non-constructable base for VectorND<> templates. Methods applicable to all dimensions are implemented in this template, while dimension specific stuff is
     * in specializations of VectorND<>. This solution uses static polymorphism and does not require any extra overhead of virtual functions and runtime type
     * identification.
     * @tparam dimensions dimensionality of the vector.
     * @tparam Element base type of vector elements.
     * @tparam ChildTemplate template inheriting this one (CRTIP).
     */
    template<int dimensions, typename Element, template<int, typename> class ChildTemplate>
    class VectorND_common
    {
        static_assert(dimensions > 0, "VectorND must have at least one dimension");
    private:
        typedef ChildTemplate<dimensions, Element> ChildType;

    public:
        typedef Element ElementType;    //!< base type for vector elements.
        typedef Element DistanceType;   //!< return type of the distance function.
        typedef Eigen::Matrix<Element, dimensions, 1> EigenType;    //!< type of the underlying Eigen object.

        template<int, int, typename> friend
        class Matrix;

        //! Default destructor.
        ~VectorND_common() = default;

        //! Casting to different ElementType.
        template<typename NewElement>
        ChildTemplate<dimensions, NewElement> cast() const
        {
            return ChildTemplate<dimensions, NewElement>(elements.template cast<NewElement>());
        }

        //! Reference to underlying Eigen data.
        EigenType &data()
        {
            return elements;
        }

        //! Const reference to underlying Eigen data.
        const EigenType &data() const
        {
            return elements;
        }

        //! Tests whether Not-a-Numbers are present in the vector.
        /*!
         * Only works for floating point ElementType, since no other type is guaranteed to have NaN representation.
         * @return true if at least one NaN is present, false otherwise.
         */
        [[nodiscard]] bool hasNaN() const
        {
            static_assert(std::is_floating_point<Element>::value, "VectorND::hasNaN() only works on floating point ElementType.");
            return elements.hasNaN();
        }

        //! Return copy of the i-th element.
        /*!
         *
         * @param i index of the element of interest.
         * @return copy of the i-th element.
         */
        Element getElement(size_t i) const
        {
            return elements[i];
        }

        //! Sets the i-th element to the given value.
        /*!
         *
         * @param i index of the element to be set.
         * @param value new value of the i-th element.
         */
        void setElement(size_t i, Element value)
        {
            elements[i] = value;
        }

        //! Index based access to the vector elements.
        /*!
         *
         * @param i index of the element of interest.
         * @return reference to the i-th element.
         */
        Element &operator[](size_t i)
        {
            return elements[i];
        }

        //! Const reference to the i-th element.
        /*!
         *
         * @param i index of the element of interest.
         * @return const reference to the i-th element.
         */
        const Element &operator[](size_t i) const
        {
            return elements[i];
        }

        //! Length of the vector.
        /*!
         * Requires std::sqrt(), use lengthSquared() where applicable and speed is important (e.g. comparison of many vector lengths against threshold).
         * @return length of the vector.
         */
        DistanceType length() const
        {
            return std::sqrt(dotProduct(*this, *this));
        }

        //! Squared length of the vector.
        /*!
         *
         * @return squared length of the vector.
         */
        DistanceType lengthSquared() const
        {
            return dotProduct(*this, *this);
        }

        //! Normalizes the vector to the unit length.
        void normalize()
        {
            elements.normalize();
        }

        //! Returns a normalized version of *this vector.
        /*!
         *
         * @return vector of the same type and direction, but with unit length.
         */
        ChildType normalized() const
        {
            return ChildType(elements.normalized());
        }

        //! Sum of two vectors.
        /*!
         *
         * @param v vector to be summed with *this.
         * @return a new vector representing the sum.
         */
        ChildType operator+(const VectorND_common<dimensions, Element, ChildTemplate> &v) const
        {
            ChildType out(elements);
            out.elements += v.elements;
            return out;
        }

        //! Adds a vector to *this.
        /*!
         *
         * @param v vector to be summed with *this.
         * @return a reference to *this.
         */
        ChildType &operator+=(const VectorND_common<dimensions, Element, ChildTemplate> &v)
        {
            elements += v.elements;
            return childThis();
        }

        //! Subtraction of two vectors.
        /*!
         *
         * @param v vector to be subtracted from *this.
         * @return a new vector representing the subtraction.
         */
        ChildType operator-(const VectorND_common<dimensions, Element, ChildTemplate> &v) const
        {
            ChildType out(elements);
            out.elements -= v.elements;
            return out;
        }

        //! Subtracts a vector from *this.
        /*!
         *
         * @param v vector to be subtracted.
         * @return a reference to *this.
         */
        ChildType &operator-=(const VectorND_common<dimensions, Element, ChildTemplate> &v)
        {
            elements -= v.elements;
            return childThis();
        }

        //! Multiplication by a scalar factor.
        /*!
         *
         * @param factor the multiplier.
         * @return a new vector with the result of multiplication.
         */
        ChildType operator*(Element factor) const
        {
            ChildType out(elements);
            out.elements *= factor;
            return out;
        }

        //! Multiplies *this with a scalar factor.
        /*!
         *
         * @param factor the multiplier.
         * @return a reference to *this.
         */
        ChildType &operator*=(Element factor)
        {
            elements *= factor;
            return childThis();
        }

        //! Division by a scalar divisor.
        /*!
         *
         * @param divisor the divisor.
         * @return a new vector with the result of division.
         */
        ChildType operator/(Element divisor) const
        {
            ChildType out(elements);
            out.elements /= divisor;
            return out;
        }

        //! Divides *this by a scalar divisor.
        /*!
         *
         * @param divisor the divisor.
         * @return a reference to *this.
         */
        ChildType &operator/=(Element divisor)
        {
            elements /= divisor;
            return childThis();
        }

        //! Equality test operator.
        /*!
         *
         * @param v vector to be tested.
         * @return true if all elements exactly match the counterparts with same index, false otherwise.
         */
        bool operator==(const ChildType &v) const
        {
            return elements == v.elements;
        }

        //! Inequality test operator.
        /*!
         *
         * @param v vector to be tested.
         * @return true if at least one element does not match its counterpart with the same index, false otherwise.
         */
        bool operator!=(const ChildType &v) const
        {
            return elements != v.elements;
        }

        //! Dot (inner) product of two vectors.
        /*!
         *
         * @param v1 first operand.
         * @param v2 second operand.
         * @return \f$ v1 \cdot v2 \f$
         */
        static DistanceType dotProduct(const VectorND_common<dimensions, Element, ChildTemplate> &v1, const VectorND_common<dimensions, Element, ChildTemplate> &v2)
        {
            DistanceType dp = v1.elements.dot(v2.elements);
            return dp;
        }

        //! Smaller angle between two vectors.
        /*!
         *
         * @param v1 first vector.
         * @param v2 second vector.
         * @return smaller angle between the vectors.
         */
        static ElementType angleShortest(const VectorND_common<dimensions, Element, ChildTemplate> &v1, const VectorND_common<dimensions, Element, ChildTemplate> &v2)
        {
            DistanceType dp = dotProduct(v1, v2);
            return std::acos(dp / (v1.length() * v2.length()));
        }

        //! Dimensionality of the vector.
        static constexpr int dimensionality() { return dimensions; }

        //! Euclidean distance between two vectors.
        /*!
         * Requires std::sqrt(), use distanceSquared() where applicable and speed is important (e.g. comparison of many vector distances against threshold).
         * @param v1 first vector.
         * @param v2 second vector.
         * @return length of v1 - v2.
         */
        static DistanceType distance(const VectorND_common<dimensions, Element, ChildTemplate> &v1, const VectorND_common<dimensions, Element, ChildTemplate> &v2)
        {
            return (v1 - v2).length();
        }

        //! Squared euclidean distance between two vectors.
        /*!
         *
         * @param v1 first vector.
         * @param v2 second vector.
         * @return squared length of v1 - v2.
         */
        static DistanceType distanceSquared(const VectorND_common<dimensions, Element, ChildTemplate> &v1, const VectorND_common<dimensions, Element, ChildTemplate> &v2)
        {
            return (v1 - v2).lengthSquared();
        }

        //! Scalar projection of one vector on another.
        /*!
         * Length of \p proj projected on \p on. If \p on is a unit vector, use scalarProjectionOnUnit() for increased speed.
         * @param proj vector to be projected.
         * @param on vector on which \p proj is projected.
         * @return length of the projection.
         */
        static DistanceType scalarProjection(const VectorND_common<dimensions, Element, ChildTemplate> &proj, const VectorND_common<dimensions, Element, ChildTemplate> &on)
        {
            return dotProduct(proj, on) / on.length();
        }

        //! Scalar projection of a vector on a unit vector.
        /*!
         * Length of \p proj projected on \p on. No checks of \p on length are performed, gives unexpected results if the condition is not met.
         * @param proj vector to be projected.
         * @param on unit vector on which \p proj is projected.
         * @return length of the projection.
         */
        static DistanceType scalarProjectionOnUnit(const VectorND_common<dimensions, Element, ChildTemplate> &proj, const VectorND_common<dimensions, Element, ChildTemplate> &on)
        {
            return dotProduct(proj, on);
        }

        //! Vector projection of one vector on another.
        /*!
         * Computes projection of \p proj on \p on. If \p on is a unit vector, use vectorProjectionOnUnit() for increased speed.
         * @param proj vector to be projected.
         * @param on vector on which \p proj is projected.
         * @return length of the projection.
         */
        static ChildType vectorProjection(const VectorND_common<dimensions, Element, ChildTemplate> &proj, const VectorND_common<dimensions, Element, ChildTemplate> &on)
        {
            return on * (dotProduct(proj, on) / dotProduct(on, on));
        }

        //! Vector projection of a vector on a unit vector.
        /*!
         * Computes projection of \p proj on unit vector \p on. No checks of \p on length are performed, gives unexpected results if the condition is not met.
         * @param proj vector to be projected.
         * @param on vector on which \p proj is projected.
         * @return length of the projection.
         */
        static ChildType vectorProjectionOnUnit(const VectorND_common<dimensions, Element, ChildTemplate> &proj, const VectorND_common<dimensions, Element, ChildTemplate> &on)
        {
            return on * dotProduct(proj, on);
        }

        //! Unary minus for reversion of the vector's direction.
        /*!
         *
         * @param v vector to be reversed.
         * @return a new reversed copy of *this.
         */
        friend ChildType operator-(const VectorND_common<dimensions, Element, ChildTemplate> &v)
        {
            return v * -1;
        }

        //! Returns a new vector with all elements initialized to zero.
        /*!
         *
         * @return a new vector of zeros.
         */
        static ChildType zeros()
        {
            ChildType ret;
            ret.elements = EigenType::Zero();
            return ret;
        }

        //! Returns a new vector with all elements initialized to one.
        /*!
         *
         * @return a new vector of ones.
         */
        static ChildType ones()
        {
            ChildType ret;
            ret.elements = EigenType::Constant(static_cast<Element>(1));
            return ret;
        }

        //! For floating point ElementType returns a new vector with all elements initialized to Not-a-Number.
        /*!
         *
         * @return a new vector of NaNs.
         */
        static ChildType nan()
        {
            static_assert(std::is_floating_point<Element>::value, "Only floating point types support NaN.");
            ChildType ret;
            ret.elements = EigenType::Constant(std::numeric_limits<Element>::quiet_NaN());
            return ret;
        }

        //! Return a new vector with all elements initialized by the user-supplied random generator.
        /*!
         * Randomness is fully specified by the callable object \p el_rnd_gen, in fact it does not have to be random at all.
         * @tparam ElRndSrc type of the random generating object.
         * @param el_rnd_gen callable object returning ElementType on invocation.
         * @return a new vector initialized by \p el_rnd_gen.
         */
        template<class ElRndSrc>
        static ChildType random(const ElRndSrc &el_rnd_gen)
        {
            EigenType et;
            et = et.unaryExpr([&el_rnd_gen](Element) { return (Element) el_rnd_gen(); });
            return ChildType(et);
        }

    protected:
        EigenType elements;

        VectorND_common()= default;

        template<typename ...T, typename std::enable_if<sizeof...(T) == dimensions, int>::type = 0>
        explicit VectorND_common(T ...args)
        {
            initElement(args...);
        }

        VectorND_common(const VectorND_common<dimensions, Element, ChildTemplate> &v)
        {
            elements = v.elements;
        }

        explicit VectorND_common(const ChildType &v)
        {
            elements = v.elements;
        }

        explicit VectorND_common(const EigenType &ev)
        {
            elements = ev;
        }

        template<typename... T>
        size_t initElement(Element e, T ...args)
        {
            size_t index = initElement(args...);
            elements[index] = e;
            return index - 1;
        }

        size_t initElement(Element e)
        {
            elements[dimensions - 1] = e;
            return dimensions - 2;
        }

        operator ChildType() const
        {
            return ChildType(elements);
        }

    private:
        VectorND_common<dimensions, Element, ChildTemplate> &operator=(const VectorND_common<dimensions, Element, ChildTemplate> &v)
        {
            elements = v.elements;
            return *this;
        }

        VectorND_common<dimensions, Element, ChildTemplate> &operator=(const EigenType &ev)
        {
            elements = ev;
            return *this;
        }

        ChildType &childThis() { return static_cast<ChildType &>(*this); }
        ChildType &childThis() const { return static_cast<ChildType const &>(*this); }

    };

    //! Scalar multiplication.
    /*!
     * Multiplication of a vector by a scalar multiplier from the left side.
     * @tparam dimensions dimensionality of the vector.
     * @tparam Element ElementType of the vector.
     * @tparam Child child template (CRTIP).
     * @param factor scalar multiplier.
     * @param v vector to be multiplied.
     * @return the multiplied vector.
     */
    template<int dimensions, typename Element, template<int, typename> class Child>
    inline Child<dimensions, Element> operator*(Element factor, const VectorND_common<dimensions, Element, Child> &v)
    {
        return v * factor;
    }

    //! N-dimensional vector template.
    /*!
     * General implementation of vectors of any dimension. For dimensions of special interest (2D and 3D) there are specializations with additional functionality.
     * @tparam dimensions dimensionality of the vector.
     * @tparam Element type of the vector's elements.
     */
    template<int dimensions, typename Element>
    class VectorND : public VectorND_common<dimensions, Element, VectorND>
    {
    public:
        typedef typename VectorND_common<dimensions, Element, VectorND>::EigenType EigenType; //!< Type of underlying Eigen object.

        //! Default constructor, the vector is uninitialized.
        VectorND()= default;

        //! Copy constructor.
        VectorND(const VectorND<dimensions, Element> &v) : VectorND_common<dimensions, Element, VectorND>(v.elements) {}

        //! Construction from the underlying EigenType.
        explicit VectorND(const EigenType &e) : VectorND_common<dimensions, Element, VectorND>(e) {}

        //! Element-wise construction - number of arguments must correspond to vector's dimensionality.
        template<typename ...T, typename std::enable_if<sizeof...(T) == dimensions, int>::type = 0>
        explicit VectorND(T ...args) : VectorND_common<dimensions, Element, VectorND>(args...) {}

        //! Assignment operator.
        VectorND<dimensions, Element> &operator=(const VectorND<dimensions, Element> &v)
        {
            this->elements = v.elements;
            return *this;
        }

        //! Assigns the vector with the EigenType variable.
        VectorND<dimensions, Element> &operator=(const EigenType &ev)
        {
            this->elements = ev;
            return *this;
        }

    };

    //! Two dimensional specialization of VectorND template.
    /*!
     * Adds 2D-exclusive functions to the general implementation of an N-dimensional vector.
     * @tparam Element base type for vector elements.
     */
    template<typename Element>
    class VectorND<2, Element> : public VectorND_common<2, Element, VectorND>
    {
    public:
        typedef typename VectorND_common<2, Element, VectorND>::ElementType ElementType;       //!< base type for vector elements.
        typedef typename VectorND_common<2, Element, VectorND>::DistanceType DistanceType;     //!< return type of the distance function.
        typedef typename VectorND_common<2, Element, VectorND>::EigenType EigenType;           //!< type of the underlying Eigen object.

        //! Default constructor, the vector is uninitialized.
        VectorND()= default;

        //! Copy constructor.
        VectorND(const VectorND<2, Element> &v) : VectorND_common<2, Element, VectorND>(v.elements) {}

        //! Construction from the underlying EigenType.
        explicit VectorND(const EigenType &e) : VectorND_common<2, Element, VectorND>(e) {}

        //! Element-wise construction.
        VectorND(ElementType x, ElementType y) : VectorND_common<2, Element, VectorND> (x, y) {}

        //! Assignment operator.
        VectorND<2, Element> &operator=(const VectorND<2, Element> &v)
        {
            this->elements = v.elements;
            return *this;
        }

        //! Assigns the vector with the EigenType variable.
        VectorND<2, Element> &operator=(const EigenType &ev)
        {
            this->elements = ev;
            return *this;
        }

        //! Returns x coordinate.
        /*!
         *
         * @return return element with index 0.
         */
        [[nodiscard]] ElementType x() const { return VectorND<2, Element>::elements[0]; }

        //! Returns y coordinate.
        /*!
         *
         * @return return element with index 1.
         */
        [[nodiscard]] ElementType y() const { return VectorND<2, Element>::elements[1]; }

        //! Sets x coordinate.
        void setX(ElementType x) { VectorND<2, Element>::elements[0] = x; }

        //! Sets y coordinate.
        void setY(ElementType y) { VectorND<2, Element>::elements[1] = y; }

        //! Return angle in radians measured from the x axis to *this.
        /*!
         *
         * @return angle in radians.
         */
        [[nodiscard]] ElementType angleFromZero() const
        {
            return std::atan2(VectorND<2, Element>::elements[1], VectorND<2, Element>::elements[0]);
        }

        //! Return angle between two vectors measured in counter-clockwise direction.
        /*!
         * Due to given direction of the angle, it is not necessarily the shortest one, in other words it might exceed \f$ \pi \f$ radians.
         * @return angle in radians.
         */
        static ElementType angleCcw(const VectorND<2, Element> &from, const VectorND<2, Element> &to)
        {
            VectorND<2, Element> from_rot(-from.y(), from.x());
            ElementType dot_orig = VectorND<2, Element>::dotProduct(from, to);
            ElementType dot_rot = VectorND<2, Element>::dotProduct(from_rot, to);
            return std::atan2(dot_rot, dot_orig);
        }

        //! Magnitude of a vector resulting from cross product of \p v1 and \p v2 in 3D.
        /*!
         * Though the name is mathematically nonsense, the result of this computation is sometimes needed and one usually arrives to it from real 3D cross product properties.
         * @param v1 first operand.
         * @param v2 second operand.
         * @return magnitude of \p v1 \f$ \times \f$ \p v2.
         */
        static ElementType crossProduct(const VectorND<2, Element> &v1, const VectorND<2, Element> &v2)
        {
            return v1.x() * v2.y() - v1.y() * v2.x();
        }

        //! Returns transformed copy of the vector.
        /*!
         * @param tf the transformation to be applied.
         * @return new vector after transformation.
         */
        VectorND<2, Element> transformed(const Transformation2D<Element> &tf) const
        {
            return VectorND<2, Element>(tf.rotMat() * (*this) + tf.tr());
        }

        //! Transforms *this vector in-place.
        /*!
         *
         * @param tf the transformation to be applied.
         */
        void transform(const Transformation2D<Element> &tf)
        {
            *this = tf.rotMat() * (*this) + tf.tr();
        }

        //! Unit vector in x axis direction.
        /*!
         *
         * @return vector [1, 0]
         */
        static VectorND<2, Element> baseX() { return VectorND<2, Element>(1.0, 0); }

        //! Unit vector in y axis direction.
        /*!
         *
         * @return vector [0, 1]
         */
        static VectorND<2, Element> baseY() { return VectorND<2, Element>(0, 1.0); }

    };

    //! Three dimensional specialization of VectorND template.
    /*!
     * Adds 3D-exclusive functions to the general implementation of an N-dimensional vector.
     * @tparam Element base type for vector elements.
     */
    template<typename Element>
    class VectorND<3, Element> : public VectorND_common<3, Element, VectorND>
    {
    public:
        typedef typename VectorND_common<3, Element, VectorND>::ElementType ElementType;         //!< base type for vector elements.
        typedef typename VectorND_common<3, Element, VectorND>::DistanceType DistanceType;       //!< return type of the distance function.
        typedef typename VectorND_common<3, Element, VectorND>::EigenType EigenType;             //!< type of the underlying Eigen object.

        //! Default constructor, the vector is uninitialized.
        VectorND()= default;

        //! Copy constructor.
        VectorND(const VectorND<3, Element> &v) : VectorND_common<3, Element, VectorND>(v.elements) {}

        //! Construction from the underlying EigenType.
        explicit VectorND(const EigenType &e) : VectorND_common<3, Element, VectorND>(e) {}

        //! Element-wise construction.
        VectorND(Element x, Element y, Element z): VectorND_common<3, Element, VectorND>(x, y, z) {}

        //! Assignment operator.
        VectorND<3, Element> &operator=(const VectorND<3, Element> &v)
        {
            this->elements = v.elements;
            return *this;
        }

        //! Assigns the vector with the EigenType variable.
        VectorND<3, Element> &operator=(const EigenType &ev)
        {
            this->elements = ev;
            return *this;
        }

        //! Returns x coordinate.
        /*!
         *
         * @return return element with index 0.
         */
        Element x() const { return VectorND<3, Element>::elements[0]; }

        //! Returns y coordinate.
        /*!
         *
         * @return return element with index 1.
         */
        Element y() const { return VectorND<3, Element>::elements[1]; }

        //! Returns z coordinate.
        /*!
         *
         * @return return element with index 2.
         */
        Element z() const { return VectorND<3, Element>::elements[2]; }

        //! Sets x coordinate.
        void setX(Element x) { VectorND<3, Element>::elements[0] = x; }

        //! Sets y coordinate.
        void setY(Element y) { VectorND<3, Element>::elements[1] = y; }

        //! Sets z coordinate.
        void setZ(Element z) { VectorND<3, Element>::elements[2] = z; }

        //! Returns transformed copy of the vector.
        /*!
         * @param tf the transformation to be applied.
         * @return new vector after transformation.
         */
        VectorND<3, Element> transformed(const Transformation3D<Element> &tf) const
        {
            return VectorND<3, Element>(tf.rotMat() * (*this) + tf.tr());
        }

        //! Transforms *this vector in-place.
        /*!
         *
         * @param tf the transformation to be applied.
         */
        void transform(const Transformation3D<Element> &tf)
        {
            *this = tf.rotMat() * (*this) + tf.tr();
        }

        //! Cross product of given vectors.
        /*!
         *
         * @param v1 first operand.
         * @param v2 second operand.
         * @return result of \p v1 \f$ \times \f$ \p v2.
         */
        static VectorND<3, Element> crossProduct(const VectorND<3, Element> &v1, const VectorND<3, Element> &v2)
        {
            VectorND<3, Element> ret;
            ret.elements = v1.elements.cross(v2.elements);
            return ret;
        }

        //! Unit vector in x axis direction.
        /*!
         *
         * @return vector [1, 0, 0]
         */
        static VectorND<3, Element> baseX() { return VectorND<3, Element>(1.0, 0, 0); }

        //! Unit vector in y axis direction.
        /*!
         *
         * @return vector [0, 1, 0]
         */
        static VectorND<3, Element> baseY() { return VectorND<3, Element>(0, 1.0, 0); }

        //! Unit vector in z axis direction.
        /*!
         *
         * @return vector [0, 0, 1]
         */
        static VectorND<3, Element> baseZ() { return VectorND<3, Element>(0, 0, 1.0); }
    };

}

#endif // ROBOTICTEMPLATELIBRARY_VECTORND_H

