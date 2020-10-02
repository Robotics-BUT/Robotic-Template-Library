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

#include <gtest/gtest.h>
#include <iostream>
#include <cmath>

#include "rtl/Core.h"
#include "rtl/Transformation.h"
#include "rtl/io/StdLib.h"
#include "rtl/Test.h"

#define STRING_STREAM(x) static_cast<std::ostringstream &&>((std::ostringstream() << x)).str()

template<typename E>
void testRotation3DConsistency(const rtl::Rotation3D<E> &rot)
{
    using M = rtl::Matrix<3, 3, E>;
    Eigen::AngleAxis<E> aa(rot.rotAngle(), rot.rotAxis().data());
    M aa_mat(aa.toRotationMatrix());
    if(M::distance(rot.rotMat(), aa_mat) > rtl::test::type<M>::allowedError()) {
        ASSERT_ANY_THROW(STRING_STREAM("\tInconsistent " << rtl::test::type<rtl::Rotation3D<E>>::description()));
    }
}

template <int dim, typename E>
struct TesterTranslationInversion
{
    static void testFunction(int rep)
    {
        using V = rtl::VectorND<dim, E>;
        using T = rtl::TranslationND<dim, E>;

        std::cout << "\n" << rtl::test::type<T>::description() << " inversion test:" << std::endl;
        auto el_gen = rtl::test::Random::uniformCallable<E>((E)-1, (E)1);

        for (int i = 0; i < rep; i++)
        {
            T tr = T::random(el_gen);
            T tr_inv_inv = tr.inverted().inverted();
            E error = V::distance(tr.trVec(), tr_inv_inv.trVec());
            if (error > rtl::test::type<V>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tTranslation not the same after double inversion. Error: " << error));
            }
        }
    }
};

template<int dim, typename T>
struct TesterRotationSpecialSetRot
{
    static void testFunction(int repeat)
    {
        using V = rtl::VectorND<dim, T>;
        using M = rtl::Matrix<dim, dim, T>;
        using R = rtl::RotationND<dim, T>;
        auto el_gen =  rtl::test::Random::uniformCallable<T>((T)-1, (T)1);

        std::cout << "\n" << rtl::test::type<R>::description() << " setting rotation with two vectors. Equivalency of special and general implementation test:" << std::endl;

        for (int i = 0; i < repeat; i++)
        {
            V v1 = V::random(el_gen), v2 = V::random(el_gen);
            R rot_spec(v1, v2);
            R rot_gen;
            rot_gen.rtl::template RotationND_common<dim, T, rtl::RotationND>::setRot(v1, v2);

            T error = M::distance(rot_gen.rotMat(), rot_spec.rotMat());
            if (error > rtl::test::type<M>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tExcessive error " << error << " for vectors " << v1 << " and " << v2));
            }
            if (std::abs(rot_spec.rotAngle()) > rtl::C_PI<T>) {
                ASSERT_ANY_THROW(STRING_STREAM("\tLarger angle (" << rot_gen.rotAngle() / rtl::C_PI<T> << ") selected for the rotation."));
            }
            if constexpr (dim == 3)
                testRotation3DConsistency(rot_spec);
        }
    }
};

template <int dim, typename T>
struct TesterRotationFull
{
    static void testFunction(int rep, int div_max)
    {
        using V = rtl::VectorND<dim, T>;
        using R = rtl::RotationND<dim, T>;

        std::cout << "\n" << rtl::test::type<R>::description() << " full rotation test:" << std::endl;
        auto el_gen = rtl::test::Random::uniformCallable<T>((T)-1, (T)1);

        for (int i = 0; i < rep; i++)
        {
            V vec_orig = V::random(el_gen).normalized();
            V vec_ortho = V::random(el_gen);
            vec_ortho = (vec_ortho - vec_orig.dot(vec_ortho) * vec_orig).normalized();
            V vec_tr = V::random(el_gen), vec_tr_orig = vec_tr;
            int d = rtl::test::Random::uniformValue<int>(3, div_max);
            T angle = 2.0 * rtl::C_PI<T> / (T) d;
            R rot(vec_orig, std::cos(angle) * vec_orig + std::sin(angle) * vec_ortho);
            for (int j = 0; j < d; j++)
                vec_tr = rot(vec_tr);
            T error = V::distance(vec_tr_orig, vec_tr);
            if (error > rtl::test::type<V>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tExcessive error " << error << " after applying the rotation " << d << " times."));
            }
        }
    }
};

template <int dim, typename T>
struct TesterRotationInversion
{
    static void testFunction(int rep)
    {
        using R = rtl::RotationND<dim, T>;
        using M = rtl::Matrix<dim, dim, T>;

        std::cout << "\n" << rtl::test::type<R>::description() << " inversion test:" << std::endl;
        auto el_gen = rtl::test::Random::uniformCallable<T>((T)-1, (T)1);

        for (int i = 0; i < rep; i++)
        {
            R rot = R::random(el_gen);
            R rot_inv_inv = rot.inverted().inverted();
            T error = M::distance(rot.rotMat(), rot_inv_inv.rotMat());
            if (error > rtl::test::type<M>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tRotation not the same after double inversion. Error: " << error));
            }
            if constexpr (dim == 3)
                testRotation3DConsistency(rot_inv_inv);
        }
    }
};

template <int dim, typename T>
struct TesterRigidTfInversion
{
    static void testFunction(int rep)
    {
        using Tf = rtl::RigidTfND<dim, T>;
        using V = rtl::VectorND<dim, T>;
        using M = rtl::Matrix<dim, dim, T>;

        std::cout << "\n" << rtl::test::type<Tf>::description() << " inversion test:" << std::endl;
        auto el_gen = rtl::test::Random::uniformCallable<T>((T)-1, (T)1);

        for (int i = 0; i < rep; i++)
        {
            Tf tf = Tf::random(el_gen);
            Tf tf_inv_inv = tf.inverted().inverted();
            T error = M::distance(tf.rotMat(), tf_inv_inv.rotMat());
            if (error > rtl::test::type<M>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tRotation not the same after double inversion. Error: " << error));
            }
            error = V::distance(tf.trVec(), tf_inv_inv.trVec());
            if (error > rtl::test::type<V>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tTranslation not the same after double inversion. Error: " << error));
            }
        }
    }
};

template <int dim, typename E>
struct TesterComposition
{
    template<typename T1, typename T2>
    static void testComposition(int rep)
    {
        std::cout << "\nComposition consistency test for " << rtl::test::type<T1>::description() << " and " << rtl::test::type<T2>::description() << std::endl;

        using V = rtl::VectorND<dim, E>;
        auto el_gen = rtl::test::Random::uniformCallable<E>(-1, 1);

        for (int i = 0; i < rep; i++)
        {
            V vec = V::random(el_gen);
            T1 tr1 = T1::random(el_gen);
            T2 tr2 = T2::random(el_gen);

            auto tr_comp = tr2(tr1);
            V vec_comp = tr_comp(vec);
            V vec_tr_tr = vec.transformed(tr1).transformed(tr2);
            V vec_call_call = tr2(tr1(vec));

            if (V::distance(vec_tr_tr, vec_comp) > rtl::test::type<V>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tInconsistency between vec.transformed(tr1).transformed(tr2) and tr_comp(vec)."));
            }
            if (V::distance(vec_tr_tr, vec_call_call) > rtl::test::type<V>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tInconsistency between vec.transformed(tr1).transformed(tr2) and tr2(tr1(vec))."));
            }
            if (V::distance(vec_call_call, vec_comp) > rtl::test::type<V>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tInconsistency between tr2(tr1(vec)) and tr_comp(vec)."));
            }
            if constexpr (dim == 3 && std::is_same<T1, rtl::Rotation3D<E>>::value && std::is_same<T2, rtl::Rotation3D<E>>::value)
                testRotation3DConsistency(tr_comp);
        }
    }

    static void testFunction(int rep)
    {
        using Tr = rtl::TranslationND<dim, E>;
        using Rot = rtl::RotationND<dim, E>;
        using Rtf = rtl::RigidTfND<dim, E>;

        testComposition<Tr, Tr>(rep);
        testComposition<Tr, Rot>(rep);
        testComposition<Tr, Rtf>(rep);

        testComposition<Rot, Tr>(rep);
        testComposition<Rot, Rot>(rep);
        testComposition<Rot, Rtf>(rep);

        testComposition<Rtf, Tr>(rep);
        testComposition<Rtf, Rot>(rep);
        testComposition<Rtf, Rtf>(rep);
    }
};

template <typename T>
struct TesterRotation3DRpy
{
    static void testFunction(int rep)
    {
        using M = rtl::Matrix<3, 3, T>;
        using R = rtl::RotationND<3, T>;
        auto ang_gen = rtl::test::Random::uniformCallable(-rtl::C_PI<T>, rtl::C_PI<T>);
        std::cout << "\n" << rtl::test::type<R>::description() << " RPY constructor/getter test:" << std::endl;

        for (int i = 0; i < rep; i++)
        {
            T r = ang_gen(), p = ang_gen(), y = ang_gen(), r1, p1, y1;
            R rot_1(r, p, y);
            rot_1.rotRpy(r1, p1, y1);
            R rot_2(r1, p1, y1);
            T error = M::distance(rot_1.rotMat(), rot_2.rotMat());
            if (error > rtl::test::type<M>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tInconsistent RPY constructor/getter for: r = " << r << ", p = " << p << ", y = " << y));
            }
        }
    }
};


TEST(t_transformationxx_tests, general_test) {

    int repeat = 10;

    rtl::test::RangeTypes<TesterTranslationInversion, 1, 5, float, double> t_tr_inv(repeat);

    rtl::test::RangeTypes<TesterRotationSpecialSetRot, 2, 3, float, double> t_rot_ssr(repeat);
    rtl::test::Types<TesterRotation3DRpy, float, double> t_rot_3d_rpy(repeat);
    rtl::test::RangeTypes<TesterRotationFull, 2, 5, float, double> t_rot_full(repeat, 20);
    rtl::test::RangeTypes<TesterRotationInversion, 2, 5, float, double> t_rot_inv(repeat);

    rtl::test::RangeTypes<TesterRigidTfInversion, 2, 5, float, double> t_rig_inv(repeat);

    rtl::test::RangeTypes<TesterComposition, 2, 4, float, double> t_comp(repeat);
}



int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}