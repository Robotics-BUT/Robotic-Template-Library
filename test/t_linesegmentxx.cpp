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

#include <iostream>
#include <cmath>
#include <ctime>
#include <random>
#include <chrono>

#include "rtl/Core.h"
#include "rtl/Transformation.h"
#include "rtl/io/StdLib.h"

// LineSegment2D (original implementation) and LineSegmentND equivalency test
template <typename T>
void ls2D_lsND_lengthDistTest(unsigned int rep, T eps)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<T> dist(-1, 1);

    std::cout << "\nLineSegmentND length and dist test:" << std::endl;

    for (unsigned int i = 0; i < rep; i++)
    {
        rtl::Vector2D<T> beg(dist(generator), dist(generator)), end(dist(generator), dist(generator)), pt(dist(generator), dist(generator));
        rtl::LineSegment2D<T> ls2d(beg, end);
        rtl::LineSegmentND<2, T> lsnd(beg, end);

        T diff_length = ls2d.length() - lsnd.length();
        if(std::abs(diff_length) > eps)
            std::cout << "Length error!  Beg: "<< beg.x() << ", " << beg.y() << "  End: " << end.x() << ", " << end.y() << "  Diff: " << diff_length << std::endl;

        T diff_d_orig = ls2d.distanceToOrigin() - lsnd.distanceToOrigin();

        if(std::abs(diff_d_orig) > eps)
            std::cout << "Distance to origin error!  Beg: "<< beg.x() << ", " << beg.y() << "  End: " << end.x() << ", " << end.y() << "  Diff: " << diff_d_orig << std::endl;

        T diff_d_point = ls2d.distanceToPoint(pt) - lsnd.distanceToPoint(pt);

        if(std::abs(diff_d_point) > eps)
            std::cout << "Distance to point error!  Beg: "<< beg.x() << ", " << beg.y() << "  End: " << end.x() << ", " << end.y() << "  Point: " << pt.x() << ", " << pt.y() << "  Diff: " << diff_d_point << std::endl;
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

        if(std::abs(shift - proj) > eps)
            std::cout << "Point projection error!  Beg: "<< beg.x() << ", " << beg.y() << "  End: " << end.x() << ", " << end.y() << "  Point: " << pt.x() << ", " << pt.y() << "  Shift: " << shift << std::endl;
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
            if(std::abs(t_1 - u_1) > eps || abs(t_2 - u_2) > eps)
                std::cout << "Closest point error!" << std::endl;
    }
}

template<class L>
void transformation(unsigned int rep, typename L::ElementType eps)
{
    std::cout<<"\nTransformation test:"<<std::endl;
    using TranformationType = typename std::conditional<L::dimensionality() == 2, rtl::Transformation2D<typename L::ElementType>,
            typename std::conditional<L::dimensionality() == 3, rtl::Transformation3D<typename L::ElementType>, void>::type>::type;

    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<typename L::ElementType> rnd_element(-1, 1);
    std::uniform_real_distribution<typename L::ElementType> rnd_angle(-rtl::C_PI, rtl::C_PI);
    auto el_rnd_gen = [&generator, &rnd_element](){ return rnd_element(generator); };
    auto ang_rnd_gen = [&generator, &rnd_angle](){ return rnd_angle(generator); };

    for(unsigned int i = 0; i < rep; i++)
    {
        L l1 = L::random(el_rnd_gen);
        auto tr = TranformationType::random(ang_rnd_gen, el_rnd_gen);
        L l_tr = l1.transformed(tr);
        if (L::VectorType::distance(l_tr.direction(), (l_tr.end() - l_tr.beg()).normalized()) > eps)
            std::cout<<"\tNon-conforming direction vector." << " for " << l1 << " and " << tr  << std::endl;
        l_tr.transform(tr.inverted());
        typename L::DistanceType err = L::distance(l1, l_tr);
        if(err.combined(1, 1) > eps)
            std::cout<<"\tExcessive distance error " << err.combined(1, 1) << " for " << l1 << " and " << tr << std::endl;
    }
}

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

            if (std::abs(err) > 1.0)
                std::cout << "Inconsistent direction detected!" << std::endl;
        }
    }
}

int main()
{
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

    transformation<rtl::LineSegment2f>(repeat, err_eps_f);
    transformation<rtl::LineSegment2d>(repeat, err_eps_d);
    transformation<rtl::LineSegmentND<2, float>>(repeat, err_eps_f);
    transformation<rtl::LineSegmentND<2, double>>(repeat, err_eps_d);

    transformation<rtl::LineSegment3f>(repeat, err_eps_f);
    transformation<rtl::LineSegment3d>(repeat, err_eps_d);
    transformation<rtl::LineSegmentND<3, float>>(repeat, err_eps_f);
    transformation<rtl::LineSegmentND<3, double>>(repeat, err_eps_d);

    lsndFitHyperRect<2, float>(repeat, err_eps_f);
    lsndFitHyperRect<3, float>(repeat, err_eps_f);
    lsndFitHyperRect<2, double>(repeat, err_eps_d);
    lsndFitHyperRect<3, double>(repeat, err_eps_d);

    return 0;
}