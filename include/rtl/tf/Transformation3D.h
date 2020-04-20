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

#ifndef ROBOTICTEMPLATELIBRARY_TRANSFORMATION3D_H
#define ROBOTICTEMPLATELIBRARY_TRANSFORMATION3D_H

#include <cmath>
#include <eigen3/Eigen/Geometry>

#include "rtl/Core.h"

namespace rtl
{
    template<typename Element>
    class Transformation3D
    {
    public:
        typedef Element ElementType;
        typedef Element DistanceType;

        Transformation3D() : int_translation(0, 0, 0), int_aa(0.0f, typename Vector3D<Element>::EigenType(0, 0, 0)), int_rot(Matrix<3, 3, Element>::EigenType::Identity())
        {
            int_translation = Vector3D<Element>(0, 0, 0);
            int_rot = Matrix<3, 3, Element>::EigenType::Identity();
        }

        Transformation3D(const Transformation3D<Element> &t)
        {
            int_translation = t.int_translation;
            int_aa = t.int_aa;
            int_rot = t.int_rot;
        }

        Transformation3D(Element angle, const Vector3D<Element> &axis, const Vector3D<Element> &shift) : int_aa(angle, axis.normalized().data())
        {
            int_translation = shift;
            int_rot = int_aa.toRotationMatrix();
        }

        Transformation3D(const Quaternion<Element> &quat, const Vector3D<Element> &trans) : int_translation(trans), int_aa(quat.data())
        {
            int_rot = quat.rotMat();
        }

        Transformation3D(ElementType roll, ElementType pitch, ElementType yaw, const Vector3D<Element> &trans) : int_translation(trans), int_aa(Quaternion<ElementType>(roll, pitch, yaw).data())
        {
            int_rot = int_aa.toRotationMatrix();
        }

        ~Transformation3D() = default;

        [[nodiscard]] Vector3D<Element> tr() const { return int_translation; }

        [[nodiscard]] Element trX() const { return int_translation.x(); }

        [[nodiscard]] Element trY() const { return int_translation.y(); }

        [[nodiscard]] Element trZ() const { return int_translation.z(); }

        [[nodiscard]] Element rotAngle() const { return int_aa.angle(); }

        [[nodiscard]] Vector3D<Element> rotAxis() const { return Vector3D<Element>(int_aa.axis()); }

        [[nodiscard]] Matrix<3, 3, Element> rotMat() const { return int_rot; }

        [[nodiscard]] Quaternion<Element> rotQuaternion() const
        {
            Eigen::Quaternion<Element> q;
            q = int_aa;
            return Quaternion(q.w(), q.x(), q.y(), q.z());
        }

        // Does not necessarily return RPY used for construction of the transformation, other valid combinations representing the same rotation are possible as well!!!
        void rpy(ElementType &roll, ElementType &pitch, ElementType &yaw)
        {
            auto v = int_rot.data().eulerAngles(0, 1, 2);
            roll = v[0];
            pitch = v[1];
            yaw = v[2];
        }

        void setAngle(Element angle)
        {
            int_aa.angle() = angle;
            int_rot = int_aa.toRotationMatrix();
        }

        void setAxis(const Vector3D<Element> &axis)
        {
            int_aa.axis() = axis.normalized().data();
            int_rot = int_aa.toRotationMatrix();
        }

        void setAngleAxis(Element angle, const Vector3D<Element> &axis)
        {
            int_aa.angle() = angle;
            int_aa.axis() = axis.normalized().data();
            int_rot = int_aa.toRotationMatrix();
        }

        void setTranslation(const Vector3D<Element> &translation) { int_translation = translation; }

        void setTranslationX(Element tr_x) { int_translation.setX(tr_x); }

        void setTranslationY(Element tr_y) { int_translation.setY(tr_y); }

        void setTranslationZ(Element tr_z) { int_translation.setZ(tr_z); }

        void invert()
        {
            int_aa = int_aa.inverse();
            int_rot.transpose();
            int_translation = -int_rot * int_translation;
        }

        [[nodiscard]] Transformation3D<Element> inverted() const
        {
            Transformation3D<Element> inv;
            inv.int_aa = int_aa.inverse();
            inv.int_rot = int_rot.transposed();
            inv.int_translation = -inv.int_rot * int_translation;
            return inv;
        }

        Transformation3D<Element> &operator=(const Transformation3D<Element> &t) = default;

        template<class T>
        T operator()(const T &t) const { return t.transformed(*this); }

        Transformation3D<Element> operator()(const Transformation3D<Element> &tr) const
        {
            Transformation3D<Element> ret;
            ret.int_rot = int_rot * tr.int_rot;
            ret.int_translation = int_rot * tr.int_translation + int_translation;
            ret.int_aa.fromRotationMatrix(ret.int_rot.data());
            return ret;
        }

        static DistanceType distance(const Transformation3D<Element> &tr1, const Transformation3D<Element> &tr2, Element c_tr, Element c_rot)
        {
            DistanceType a = tr1.int_aa.angle() - tr2.int_aa.angle();
            if (a > rtl::C_PIf)
                a -= 2.0f * rtl::C_PIf;
            else if (a <= -rtl::C_PIf)
                a += 2.0f * rtl::C_PIf;
            return a * a * c_rot + (tr1.int_translation - tr2.int_translation).lengthSquared() * c_tr;
        }

        template<class AngRndSrc, class ElRndSrc>
        static Transformation3D<Element> random(const AngRndSrc &ang_rnd_gen, const ElRndSrc &el_rnd_gen)
        {
            return Transformation3D(ang_rnd_gen(), Vector3D<Element>::random(el_rnd_gen), Vector3D<Element>::random(el_rnd_gen));
        }

    private:
        Vector3D<Element> int_translation;
        Eigen::AngleAxis<Element> int_aa;
        Matrix<3, 3, Element> int_rot;
    };
}

#endif // ROBOTICTEMPLATELIBRARY_TRANSFORMATION3D_H
