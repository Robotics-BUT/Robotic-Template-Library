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
#include <ctime>
#include <utility>

#include "rtl/Core.h"
#include "rtl/io/StdLib.h"
#include "rtl/Test.h"

#include <eigen3/Eigen/Dense>

#define STRING_STREAM(x) static_cast<std::ostringstream &&>((std::ostringstream() << x)).str()

template <typename T>
struct TesterVector2DAngleCcw
{
    static void testFunction(double step)
    {

        std::cout << "\nVector2D<" << rtl::test::type<T>::description()<< ">::angle() test:" << std::endl;
        std::cout << "\tStep size: " << step << std::endl;
        std::cout << "\tAllowed error: " << rtl::test::type<T>::allowedError() << std::endl;

        rtl::Vector2D<T> base(1, 0);
        T angle;
        size_t err_cnt = 0;
        for (T i = -rtl::C_PI<T> + step; i <= rtl::C_PI<T>; i += step)
        {
            angle = rtl::Vector2D<T>::angleCcw(base, rtl::Vector2D<T>(std::cos(i), std::sin(i)));
            if (std::abs(i - angle) > rtl::test::type<T>::allowedError())
                err_cnt++;
        }
        if (err_cnt) {
            ASSERT_ANY_THROW(STRING_STREAM("\tPrecision errors: " << err_cnt));
        }
    }
};

template <typename T>
struct TesterVector2DAngleFromZero
{
    static void testFunction(double step)
    {
        std::cout << "\nVector2D<" << rtl::test::type<T>::description()<< ">::angleFromZero() test:" << std::endl;
        std::cout << "\tStep size: " << step << std::endl;
        std::cout << "\tAllowed error: " << rtl::test::type<T>::allowedError() << std::endl;

        T angle;
        size_t err_cnt = 0;
        for (T i = -rtl::C_PI<T> + step; i <= rtl::C_PI<T>; i += step)
        {
            rtl::Vector2D<T> base = rtl::Vector2D<T>(std::cos(i), std::sin(i));
            angle = base.angleFromZero();
            if (std::abs(i - angle) > rtl::test::type<T>::allowedError())
                err_cnt++;
        }
        if (err_cnt) {
            ASSERT_ANY_THROW(STRING_STREAM("\tPrecision errors: " << err_cnt));
        }
    }
};

template<int dim, typename T>
struct TesterConstruction
{
    template<std::size_t... indices>
    static rtl::VectorND<dim, T> variadicConstructedVector(std::index_sequence<indices...>)
    {
        return rtl::VectorND<dim, T>(static_cast<T>(indices)...);
    }

    static void testFunction()
    {
        using V = rtl::VectorND<dim, T>;
        std::cout << "\n" << rtl::test::type<V>::description() << " construction test:" << std::endl;
        V v;
        std::cout<<"\tDefault-constructed vector: " << v << std::endl;
        V v_copy(v);
        std::cout<<"\tCopy-constructed vector: " << v_copy << std::endl;
        V v_eigen(v.data());
        std::cout<<"\tEigen-constructed vector: " << v_eigen << std::endl;
        // Variadic template constructor test
        V v_var = variadicConstructedVector(std::make_index_sequence<dim>());
        std::cout<<"\tParameter pack constructed vector: " << v_var << std::endl;

        V v_rnd = V::random(rtl::test::Random::uniformCallable((T)-1, (T)1));
        std::cout<<"\tRandom-constructed vector: " << v_rnd << std::endl;
    }
};

template<int dim, typename T>
struct TesterElementAccess
{
    static void testFunction()
    {
        std::cout << "\n" << rtl::test::type<rtl::VectorND<dim, T>>::description() << " element acccess test:" << std::endl;
        rtl::VectorND<dim, T> v;
        v.setElement(0, static_cast<T>(0));
        for (size_t i = 0, j = 1; j < dim; i = j, j++)
            v.setElement(j, v.getElement(i) + 1);
        std::cout<<"\tSet/Get element initialization: " << v << std::endl;
    }
};

template <typename E>
struct TesterCrossProduct
{
    static void testFunction(int rep)
    {
        std::cout << "\nCross-product test - equivalency of manual and Eigen implementation." << std::endl;
        auto el_gen = rtl::test::Random::uniformCallable((E)-1, (E)1);
        using V = rtl::Vector3D<E>;
        for (int i = 0; i < rep; i++)
        {
            auto v1 = V::random(el_gen), v2 = V::random(el_gen);
            Eigen::Matrix<E, 3, 1> ev1, ev2;
            ev1[0] = v1[0];
            ev1[1] = v1[1];
            ev1[2] = v1[2];
            ev2[0] = v2[0];
            ev2[1] = v2[1];
            ev2[2] = v2[2];
            V c_rtl = v1.cross(v2);
            Eigen::Matrix<E, 3, 1> c_eig = ev1.cross(ev2);
            E error = (c_rtl[0] - c_eig[0]) * (c_rtl[0] - c_eig[0]) + (c_rtl[1] - c_eig[1]) * (c_rtl[1] - c_eig[1]) + (c_rtl[2] - c_eig[2]) * (c_rtl[2] - c_eig[2]);
            if (error > rtl::test::type<E>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tExcessive error " << error << " detected"));
            }
        }
    }
};

template<int dim, typename T>
struct TesterVectorStaticOperations
{
    static void testFunction(int rep)
    {
        using V = rtl::VectorND<dim, T>;
        std::cout << "\nStatic operations test:" << std::endl;
        auto el_gen = rtl::test::Random::uniformCallable((T)-1, (T)1);

        std::cout << "\tNaN construction:" << std::endl;
        auto v_nan = V::nan();
        if (!v_nan.hasNaN())
            std::cout << "\tNo NaNs in ::nan() initialized vector." << std::endl;

        std::cout << "\tDistance computation:" << std::endl;
        for (int i = 0; i < rep; i++)
        {
            auto v1 = V::random(el_gen);
            auto v2 = V::random(el_gen);
            T err = std::abs(V::distanceSquared(v1, v2) / V::distance(v1, v2) - (v1 - v2).length());
            if (err > rtl::test::type<T>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tExcessive error " << err));
            }
        }

        std::cout << "\tScalar projection computation:" << std::endl;
        for (int i = 0; i < rep; i++)
        {
            auto v1 = V::random(el_gen);
            auto v2 = V::random(el_gen);
            T err = std::abs(V::scalarProjection(v1, v2) - V::scalarProjectionOnUnit(v1, v2.normalized()));
            if (err > rtl::test::type<T>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tExcessive error " << err));
            }
        }

        std::cout << "\tVector projection computation:" << std::endl;
        for (int i = 0; i < rep; i++)
        {
            auto v1 = V::random(el_gen);
            auto v2 = V::random(el_gen);
            T err = (V::vectorProjection(v1, v2) - V::vectorProjectionOnUnit(v1, v2.normalized())).length();
            if (err > rtl::test::type<T>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tExcessive error " << err));
            }
        }
    }
};

template<int d1, int d2, typename T>
struct TesterOuterProduct
{
    static void testFunction(int rep)
    {
        std::cout << "\nOuter product test for " << rtl::test::type<rtl::VectorND<d1, T>>::description() << " and " << rtl::test::type<rtl::VectorND<d2, T>>::description() << ":" << std::endl;
        auto el_gen = rtl::test::Random::uniformCallable((T)-1, (T)1);

        for (int r = 0; r < rep; r++)
        {
            auto v1 = rtl::VectorND<d1, T>::random(el_gen);
            auto v2 = rtl::VectorND<d2, T>::random(el_gen);
            auto op = v1.outer(v2);

            for (int i = 0; i < d1; i++)
                for(int j = 0; j < d2; j++)
                    if (std::abs(op(i, j) - v1[i] * v2[j]) > rtl::test::type<T>::allowedError()) {
                        ASSERT_ANY_THROW(STRING_STREAM("\tExcessive error for vectors v1 = " << v1 << " and v2 = " << v2));
                    }
        }
    }
};

template<int dim, typename T>
struct TesterNormalization
{
    static void testFunction(int rep)
    {
        using V = rtl::VectorND<dim, T>;
        std::cout << "\nNormalization test:" << std::endl;
        for (int i = 0; i < rep; i++)
        {
            auto v1 = V::random(rtl::test::Random::uniformCallable((T)-1, (T)1));
            T err = std::abs(v1.normalized().length() - (T)1);
            if (err > rtl::test::type<T>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tExcessive error " << err));
            }
        }
    }
};

template<int dim, typename E>
struct TesterRigidTransformation
{
    static void testFunction(int rep)
    {
        using V = rtl::VectorND<dim, E>;
        using T = rtl::RigidTfND<dim, E>;
        std::cout << "\n" << rtl::test::type<T>::description() << " transformation test:" << std::endl;

        auto el_gen = rtl::test::Random::uniformCallable<E>((E)-1, (E)1);

        for (int i = 0; i < rep; i++)
        {
            V v1 = V::random(el_gen);
            T tr = T::random(el_gen);
            V v_tr = v1.transformed(tr);
            tr.invert();
            v_tr.transform(tr);
            E err = V::distance(v1, v_tr);
            if (err > rtl::test::type<V>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tExcessive error " << err));
            }
        }
    }
};

TEST(t_vectorxx, general_test) {

    int repeat = 1000;
    double angle_step = 0.01;

    rtl::test::RangeTypes<TesterConstruction, 1, 5, float, double> t_c;
    rtl::test::RangeTypes<TesterElementAccess, 1 ,5, float, double> t_ea;

    rtl::test::Types<TesterVector2DAngleCcw, float, double> t_vaccw(angle_step);
    rtl::test::Types<TesterVector2DAngleFromZero, float, double> t_vafz(angle_step);
    rtl::test::Types<TesterCrossProduct, float, double> t_cp(repeat);

    rtl::test::RangeTypes<TesterVectorStaticOperations, 1, 5, float, double> t_vso(repeat);
    rtl::test::RangeTypes<TesterNormalization, 1, 5, float, double> t_n(repeat);
    rtl::test::RangeRangeTypes<TesterOuterProduct, 1, 5, 1, 5, float, double> t_op(repeat);
    rtl::test::RangeTypes<TesterRigidTransformation, 2, 5, float, double> t_rtf(repeat);
}



int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
