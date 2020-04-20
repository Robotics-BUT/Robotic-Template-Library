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
#include <random>
#include <chrono>
#include <vector>

#include "rtl/Core.h"
#include "rtl/Transformation.h"

template <typename T>
void tr2D_fullRotTest(unsigned int rep, T eps)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<T> rand(-1, 1);
    std::uniform_int_distribution<unsigned int> divisor(1, 20);
    std::cout << "\nTransformation2D full rotation test:" << std::endl;

    for(unsigned int i = 0; i < rep; i++)
    {
        rtl::Vector2D<T> vec_orig(rand(generator), rand(generator)), vec_tr = vec_orig;
        unsigned int d = divisor(generator);
        rtl::Transformation2D<T> tr(2.0 * rtl::C_PI / (T)d, 0, 0);
        for(unsigned int j = 0; j < d; j++)
            vec_tr = tr(vec_tr);
        T error = rtl::Vector2D<T>::distance(vec_orig, vec_tr);
        if(error > eps)
            std::cout<< "Excessive error " << error << " after applying the transformation " << d << " times." <<std::endl;
    }
}

template <typename T>
void tr2D_forwardBackwardTest(unsigned int rep, T eps)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<T> rnd_float(-1, 1);
    std::uniform_int_distribution<unsigned int> tr_nr(1, 20);
    std::uniform_real_distribution<T> rnd_angle(-rtl::C_PI, rtl::C_PI);
    std::cout << "\nTransformation2D forward-backward transformation test:" << std::endl;

    for(unsigned int i = 0; i < rep; i++)
    {
        rtl::Vector2D<T> vec_orig(rnd_float(generator), rnd_float(generator)), vec_tr = vec_orig;
        unsigned int n = tr_nr(generator);
        std::vector<rtl::Transformation2D<T>> trs;

        for(unsigned int j = 0; j < n; j++)
        {
            rtl::Transformation2D<T> tr(rnd_angle(generator), rnd_float(generator), rnd_float(generator));
            vec_tr = tr(vec_tr);
            trs.push_back(tr);
        }
        for(unsigned int j = n - 1; j < n; j--)
            vec_tr = trs[j].inverted()(vec_tr);
        T error = rtl::Vector2D<T>::distance(vec_orig, vec_tr);
        if(error > eps)
            std::cout<< "Excessive error " << error << " after applying " << n << " transformations." <<std::endl;
    }
}

template <typename T>
void tr3D_fullRotTest(unsigned int rep, T eps)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<T> rand(-1, 1);
    std::uniform_int_distribution<unsigned int> divisor(1, 20);
    std::cout << "\nTransformation3D full rotation test:" << std::endl;

    for(unsigned int i = 0; i < rep; i++)
    {
        rtl::Vector3D<T> vec_orig(rand(generator), rand(generator), rand(generator)), vec_tr = vec_orig;
        unsigned int d = divisor(generator);
        rtl::Transformation3D<T> tr(2.0 * rtl::C_PI / (T)d, rtl::Vector3D<T>(rand(generator), rand(generator), rand(generator)), rtl::Vector3D<T>::zeros());
        for(unsigned int j = 0; j < d; j++)
            vec_tr = tr(vec_tr);
        T error = rtl::Vector3D<T>::distance(vec_orig, vec_tr);
        if(error > eps)
            std::cout<< "Excessive error " << error << " after applying the transformation " << d << " times." <<std::endl;
    }
}

template <typename T>
void tr3D_forwardBackwardTest(unsigned int rep, T eps)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<T> rnd_float(-1, 1);
    std::uniform_int_distribution<unsigned int> tr_nr(1, 20);
    std::uniform_real_distribution<T> rnd_angle(-rtl::C_PI, rtl::C_PI);
    std::cout << "\nTransformation3D forward-backward transformation test:" << std::endl;

    for(unsigned int i = 0; i < rep; i++)
    {
        rtl::Vector3D<T> vec_orig(rnd_float(generator), rnd_float(generator), rnd_float(generator)), vec_tr = vec_orig;
        unsigned int n = tr_nr(generator);
        std::vector<rtl::Transformation3D<T>> trs;

        for(unsigned int j = 0; j < n; j++)
        {
            rtl::Transformation3D<T> tr(rnd_angle(generator), rtl::Vector3D<T>(rnd_float(generator), rnd_float(generator), rnd_float(generator)), rtl::Vector3D<T>(rnd_float(generator), rnd_float(generator), rnd_float(generator)));
            vec_tr = tr(vec_tr);
            trs.push_back(tr);
        }
        for(unsigned int j = n - 1; j < n; j--)
            vec_tr = trs[j].inverted()(vec_tr);
        T error = rtl::Vector3D<T>::distance(vec_orig, vec_tr);
        if(error > eps)
            std::cout<< "Excessive error " << error << " after applying " << n << " transformations." <<std::endl;
    }
}
template <typename T>
void tr3D_rpy(unsigned int rep, T eps)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<T> rnd_angle(-rtl::C_PI, rtl::C_PI);
    auto ang_gen = [&generator, &rnd_angle](){ return rnd_angle(generator); };
    rtl::Vector3D<T> tr;

    std::cout << "\nTransformation3D RPY constructor/getter test:" << std::endl;

    for (unsigned int i = 0; i < rep; i++)
    {
        T r = ang_gen(), p = ang_gen(), y = ang_gen(), r1, p1, y1;
        rtl::Transformation3D<T> t1(r, p, y, tr);
        t1.rpy(r1, p1, y1);
        rtl::Transformation3D<T> t2(r1, p1, y1, tr);
        T dist = t1.rotAngle() - t2.rotAngle();
        if (std::abs(dist) > eps)
            std::cout<<"\t\tInconsistent RPY constructor/getter for: r = " << r <<", p = " << p << ", y = " << y << std::endl;
    }
}

int main()
{
    unsigned int repeat = 1000;
    float err_eps_f = 0.00001f;
    double err_eps_d = 0.000000001;

    // TransformationXX tests
    tr3D_rpy<float>(repeat, err_eps_f);
    tr3D_rpy<double>(repeat, err_eps_d);

    tr2D_fullRotTest<float>(repeat, err_eps_f);
    tr2D_fullRotTest<double >(repeat, err_eps_d);

    tr2D_forwardBackwardTest<float>(repeat, err_eps_f);
    tr2D_forwardBackwardTest<double>(repeat, err_eps_d);

    tr3D_fullRotTest<float>(repeat, err_eps_f);
    tr3D_fullRotTest<double>(repeat, err_eps_d);

    tr3D_forwardBackwardTest<float>(repeat, err_eps_f);
    tr3D_forwardBackwardTest<double>(repeat, err_eps_d);

    return 0;
}