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
#include <random>
#include <chrono>

#include "rtl/Core.h"
#include "rtl/Transformation.h"
#include "rtl/Test.h"
#include "rtl/io/StdLib.h"

#define STRING_STREAM(x) static_cast<std::ostringstream &&>((std::ostringstream() << x)).str()

// LineSegment2D (original implementation) and LineSegmentND equivalency test
template <typename T>
void ls2D_lsND_lengthDistTest(unsigned int rep, T eps)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<T> dist(-1, 1);

    std::cout << "\nLineSegmentND length and dist test:" << std::endl;

    for (unsigned int i = 0; i < rep; i++) {
        rtl::Vector2D<T> beg(dist(generator), dist(generator)), end(dist(generator), dist(generator)), pt(
                dist(generator), dist(generator));
        rtl::LineSegment2D<T> ls2d(beg, end);
        rtl::LineSegmentND<2, T> lsnd(beg, end);

        T diff_length = ls2d.length() - lsnd.length();
        if (std::abs(diff_length) > eps) {
            ASSERT_ANY_THROW(STRING_STREAM(
                    "Length error!  Beg: " << beg.x() << ", " << beg.y() << "  End: " << end.x() << ", " << end.y()
                                           << "  Diff: " << diff_length));
        }

        T diff_d_orig = ls2d.distanceToOrigin() - lsnd.distanceToOrigin();

        if (std::abs(diff_d_orig) > eps) {
            ASSERT_ANY_THROW(STRING_STREAM(
                    "Distance to origin error!  Beg: " << beg.x() << ", " << beg.y() << "  End: " << end.x() << ", "
                                                       << end.y() << "  Diff: " << diff_d_orig));
        }

        T diff_d_point = ls2d.distanceToPoint(pt) - lsnd.distanceToPoint(pt);

        if (std::abs(diff_d_point) > eps) {
            ASSERT_ANY_THROW(STRING_STREAM(
                    "Distance to point error!  Beg: " << beg.x() << ", " << beg.y() << "  End: " << end.x() << ", "
                                                      << end.y() << "  Point: " << pt.x() << ", " << pt.y()
                                                      << "  Diff: " << diff_d_point));
        }
    }
}

// LineSegmentND projection tests
template <typename T>
void lsndProjection(unsigned int rep, T eps)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<T> dist(-1, 1);

    std::cout << "\nLineSegmentND projection test:" << std::endl;

    for (unsigned int i = 0; i < rep; i++)
    {
        rtl::Vector2D<T> beg(dist(generator), dist(generator)), end(dist(generator), dist(generator));
        rtl::LineSegment2D<T> lsnd(beg, end);
        T shift = 2 * dist(generator);

        rtl::Vector2D<T> pt = lsnd.beg() + shift * lsnd.direction() + 2 * dist(generator) * lsnd.normal();
        T proj = lsnd.scalarProjectionUnit(pt);

        if(std::abs(shift - proj) > eps) {
            ASSERT_ANY_THROW(STRING_STREAM("Point projection error!  Beg: "<< beg.x() << ", " << beg.y() << "  End: " << end.x() << ", " << end.y() << "  Point: " << pt.x() << ", " << pt.y() << "  Shift: " << shift));
        }
    }
}

// LineSegmentND closest point test
template <typename T>
void lsndClosestPoint(unsigned int rep, T eps)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<T> dist(-1, 1);

    std::cout << "\nLineSegmentND closest point test:" << std::endl;

    for (unsigned int i = 0; i < rep; i++)
    {
        rtl::Vector2D<T> beg1(dist(generator), dist(generator)), end1(dist(generator), dist(generator)), beg2(dist(generator), dist(generator)), end2(dist(generator), dist(generator));
        rtl::LineSegment2D<T> ls2d_1(beg1, end1);
        rtl::LineSegment2D<T> ls2d_2(beg2, end2);

        T t_1, t_2, u_1, u_2;
        ls2d_1.closestPoint(ls2d_2, t_1);
        ls2d_2.closestPoint(ls2d_1, t_2);
        if (rtl::LineSegment2D<T>::closestPoint(ls2d_1, ls2d_2, u_1, u_2))
            if(std::abs(t_1 - u_1) > eps || abs(t_2 - u_2) > eps) {
                ASSERT_ANY_THROW(STRING_STREAM("Closest point error!" ));
            }
    }
}

template<int dim, typename E>
struct TesterRigidTransformation
{
    static void testFunction(int rep)
    {
        using V = rtl::VectorND<dim, E>;
        using L = rtl::LineSegmentND<dim, E>;
        using T = rtl::RigidTfND<dim, E>;
        std::cout << "\n" << rtl::test::type<T>::description() << " transformation test:" << std::endl;

        auto el_gen = rtl::test::Random::uniformCallable<E>((E)-1, (E)1);

        for (int i = 0; i < rep; i++)
        {
            L l1 = L::random(el_gen);
            auto tr = T::random(el_gen);
            L l_tr = l1.transformed(tr);
            if (V::distance(l_tr.direction(), (l_tr.end() - l_tr.beg()).normalized()) > rtl::test::type<V>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM("\tNon-conforming direction vector." << " for " << l1 << " and " << tr));
            }
            l_tr.transform(tr.inverted());
            if (V::distance(l1.beg(), l_tr.beg()) > rtl::test::type<V>::allowedError() ||
                    V::distance(l1.end(), l_tr.end()) > rtl::test::type<V>::allowedError() ||
                    V::distance(l1.direction(), l_tr.direction()) > rtl::test::type<V>::allowedError()) {
                ASSERT_ANY_THROW(STRING_STREAM(
                        "\tExcessive distance error for " << l1 << " and " << tr << "in forward-backward test."));
            }
        }
    }
};

// LineSegmentND hyperrect fitting
template <size_t dim, typename T>
void lsndFitHyperRect(unsigned int rep, T )
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<T> rnd_element(-1, 1);
    auto lambda_el_gen = [&generator, &rnd_element](){ return rnd_element(generator); };

    std::cout << "\nLineSegmentND hyperrect test:" << std::endl;

    for (unsigned int i = 0; i < rep; i++)
    {
        auto ls = rtl::LineSegmentND<dim, T>::random(lambda_el_gen);
        auto c1 = rtl::VectorND<dim, T>::random(lambda_el_gen), c2 = rtl::VectorND<dim, T>::random(lambda_el_gen);

        if (ls.fitToHyperRect(c1, c2))
        {
            auto err = ((ls.end() - ls.beg()).normalized() - ls.direction()).length();

            if (std::abs(err) > 1.0) {
                ASSERT_ANY_THROW(STRING_STREAM("Inconsistent direction detected!"));
            }
        }
    }
}

TEST(t_linesegmentxx_test, general_test) {

    unsigned int repeat = 1000;
    float err_eps_f = 0.001f;
    double err_eps_d = 0.0000001;

    // LineSegmentXX tests
    ls2D_lsND_lengthDistTest<float>(repeat, err_eps_f);
    ls2D_lsND_lengthDistTest<double >(repeat, err_eps_d);

    lsndProjection<float>(repeat, err_eps_f);
    lsndProjection<double >(repeat, err_eps_d);

    lsndClosestPoint<float>(repeat, 0.01f);
    lsndClosestPoint<double>(repeat, 0.0001f);

    rtl::test::RangeTypes<TesterRigidTransformation, 2, 5, float, double> t_rtf(100);

    lsndFitHyperRect<2, float>(repeat, err_eps_f);
    lsndFitHyperRect<3, float>(repeat, err_eps_f);
    lsndFitHyperRect<2, double>(repeat, err_eps_d);
    lsndFitHyperRect<3, double>(repeat, err_eps_d);
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}