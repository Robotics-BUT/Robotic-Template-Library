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

#ifndef ROBOTICTEMPLATELIBRARY_TRANSFORMATION2D_H
#define ROBOTICTEMPLATELIBRARY_TRANSFORMATION2D_H

#include <cmath>
#include "rtl/Core.h"

namespace rtl
{
    template<typename Element>
    class Transformation2D
    {
    public:
        typedef Element ElementType;
        typedef Element DistanceType;

        Transformation2D()
        {
            int_translation = Vector2D<Element>(0, 0);
            int_rot = Matrix<2, 2, Element>::EigenType::Identity();
        }

        Transformation2D(const Transformation2D &t)
        {
            int_translation = t.int_translation;
            int_rot = t.int_rot;
        }

        Transformation2D(Element angle, Element x_shift, Element y_shift)
        {
            int_translation = Vector2D<Element>(x_shift, y_shift);
            int_rot(0, 0) = int_rot(1, 1) = std::cos(angle);
            int_rot(1, 0) = std::sin(angle);
            int_rot(0, 1) = -int_rot(1, 0);
        }

        Transformation2D(Element angle, const Vector2D<Element> &shift)
        {
            int_translation = shift;
            int_rot(0, 0) = int_rot(1, 1) = std::cos(angle);
            int_rot(1, 0) = std::sin(angle);
            int_rot(0, 1) = -int_rot(1, 0);
        }

        ~Transformation2D() = default;

        Vector2D<Element> tr() const { return int_translation; }

        Element trX() const { return int_translation.x(); }

        Element trY() const { return int_translation.y(); }

        Element rotA() const { return std::atan2(int_rot(1, 0), int_rot(0, 0)); }

        Element rotCos() const { return int_rot(0, 0); }

        Element rotSin() const { return int_rot(1, 0); }

        Matrix<2, 2, Element> rotMat() const { return int_rot; }

        void setAngle(Element angle)
        {
            int_rot(0, 0) = int_rot(1, 1) = std::cos(angle);
            int_rot(1, 0) = std::sin(angle);
            int_rot(0, 1) = -int_rot(1, 0);
        }

        void setTranslation(Vector2D<Element> translation) { int_translation = translation; }

        void setTranslationX(Element tr_x) { int_translation.setX(tr_x); }

        void setTranslationY(Element tr_y) { int_translation.setY(tr_y); }

        Transformation2D inverted() const
        {
            return Transformation2D(-int_rot(1, 0),
                                    int_rot(0, 0),
                                    -int_rot(0, 0) * int_translation.x() - int_rot(1, 0) * int_translation.y(),
                                    int_rot(1, 0) * int_translation.x() - int_rot(0, 0) * int_translation.y());
        }

        Transformation2D &operator=(const Transformation2D &t)
        {
            int_translation = t.int_translation;
            int_rot = t.int_rot;
            return *this;
        }

        Transformation2D operator*(const Transformation2D &first) const
        {
            return Transformation2D(int_rot(1, 0) * first.int_rot(0, 0) + first.int_rot(1, 0) * int_rot(0, 0),
                                    first.int_rot(0, 0) * int_rot(0, 0) - first.int_rot(1, 0) * int_rot(1, 0),
                                    int_rot(0, 0) * first.int_translation.x() -
                                    int_rot(1, 0) * first.int_translation.y() +
                                    int_translation.x(),
                                    int_rot(1, 0) * first.int_translation.x() +
                                    int_rot(0, 0) * first.int_translation.y() +
                                    int_translation.y());
        }

        template<class T>
        T operator()(const T &t) const { return t.transformed(*this); }

        Transformation2D<Element> operator()(const Transformation2D &tr) const
        {
            Transformation2D<Element> ret;
            ret.int_rot = int_rot * tr.int_rot;
            ret.int_translation = int_rot * tr.int_translation + int_translation;
            return ret;
        }

        static Vector2D<Element> rotated(const Vector2D<Element> &v, Element s, Element c)
        {
            return Vector2D<Element>(c * v[0] - s * v[1], s * v[0] + c * v[1]);
        }

        static Vector2D<Element> rotated(const Vector2D<Element> &v, Element angle)
        {
            return rotated(v, std::sin(angle), std::cos(angle));
        }

        static DistanceType distance(const Transformation2D &tr1, const Transformation2D &tr2, DistanceType c_tr, DistanceType c_rot)
        {
            DistanceType a = std::atan2(tr1.int_rot(1, 0), tr1.int_rot(0, 0)) - std::atan2(tr2.int_rot(1, 0), tr2.int_rot(0, 0));
            if (a > rtl::C_PI)
                a -= 2.0 * rtl::C_PI;
            else if (a <= -rtl::C_PI)
                a += 2.0 * rtl::C_PI;
            return a * a * c_rot + (tr1.int_translation - tr2.int_translation).lengthSquared() * c_tr;
        }

        template<class AngRndSrc, class ElRndSrc>
        static Transformation2D<Element> random(const AngRndSrc &ang_rnd_gen, const ElRndSrc &el_rnd_gen)
        {
            return Transformation2D(ang_rnd_gen(), Vector2D<Element>::random(el_rnd_gen));
        }

    private:
        Transformation2D(Element angle_sin, Element angle_cos, Element x_shift, Element y_shift)
        {
            int_translation = Vector2D<Element>(x_shift, y_shift);

            int_rot(0, 0) = angle_cos;
            int_rot(0, 1) = -angle_sin;
            int_rot(1, 0) = angle_sin;
            int_rot(1, 1) = angle_cos;
        }

        Vector2D<Element> int_translation;
        Matrix<2, 2, Element> int_rot;
    };
}
#endif // ROBOTICTEMPLATELIBRARY_TRANSFORMATION2D_H
