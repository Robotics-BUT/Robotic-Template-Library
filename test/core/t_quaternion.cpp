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
#include <random>
#include <chrono>

#include "rtl/Core.h"

#define STRING_STREAM(x) static_cast<std::ostringstream &&>((std::ostringstream() << x)).str()

template<typename E>
void coutQuat(const rtl::Quaternion<E> &q)
{
    std::cout<<"\t"<<q.w()<<" "<<q.x()<<"i "<<q.y()<<"j "<<q.z()<<"k"<<std::endl;
}

template<typename E>
void quaternionConstruction(size_t repeat, E eps)
{
    std::cout<<"\nTesting Quaternion construction and element access/modification:"<<std::endl;
    rtl::Quaternion<E> q1;
    std::cout<<"\tDefault constructor:"<<std::endl;
    coutQuat(q1);
    rtl::Quaternion<E> q2(q1.data());
    std::cout<<"\tEigenType constructor:"<<std::endl;
    coutQuat(q2);
    rtl::Quaternion<E> q3(1, 2, 3, 4);
    std::cout<<"\tElement-wise construction:"<<std::endl;
    coutQuat(q3);
    rtl::Quaternion<E> q4(q3);
    std::cout<<"\tCopy constructor:"<<std::endl;
    coutQuat(q4);
    rtl::Quaternion<E> q5(1.02586f, rtl::Vector3D<E>(1, 1, 1));
    std::cout<<"\tAngle axis constructor:"<<std::endl;
    coutQuat(q5);
    rtl::Quaternion<E> q6(rtl::Vector3D<E>(1, 0, 0), rtl::Vector3D<E>(0, 1, 0));
    std::cout<<"\tVector to vector constructor:"<<std::endl;
    coutQuat(q6);

    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<E> rnd_element(-1, 1);
    std::uniform_real_distribution<E> rnd_angle(-rtl::C_PI<E>, rtl::C_PI<E>);

    auto lambda_el_gen = [&generator, &rnd_element](){ return rnd_element(generator); };
    auto lambda_ang_gen = [&generator, &rnd_angle](){ return rnd_angle(generator); };

    auto q7 = rtl::Quaternion<E>::random(lambda_el_gen);
    std::cout<<"\tElement-wise random construction:"<<std::endl;
    coutQuat(q7);
    auto q8 = rtl::Quaternion<E>::random(lambda_ang_gen, lambda_el_gen);
    std::cout<<"\tAngle-axis random construction:"<<std::endl;
    coutQuat(q8);
    std::cout<<"\tRoll-pitch-yaw construction:"<<std::endl;
    for (size_t i = 0; i < repeat; i++)
    {
        E r = lambda_ang_gen(), p = lambda_ang_gen(), y = lambda_ang_gen(), r1, p1, y1;
        rtl::Quaternion<E> q9(r, p, y);
        q9.rpy(r1, p1, y1);
        rtl::Quaternion<E> q10(r1, p1, y1);
        E dist = rtl::Quaternion<E>::distance(q9, q10);
        if (dist > eps && dist - 2.0 > eps) {
            ASSERT_ANY_THROW(STRING_STREAM("\t\tInconsistent RPY constructor/getter for: r = " << r << ", p = " << p << ", y = " << y));
        }
    }
}

template<typename E>
void quaternionArithmetic(unsigned int rep, E eps)
{
    std::cout<<"\nTesting base Quaternion arithmetic operators:"<<std::endl;

    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<E> rnd_element(-1, 1);
    auto lambda_el_gen = [&generator, &rnd_element](){ return rnd_element(generator); };

    for(unsigned int i = 0; i < rep; i++)
    {
        rtl::Quaternion q1 = rtl::Quaternion<E>::random(lambda_el_gen), q2 = rtl::Quaternion<E>::random(lambda_el_gen);
        auto q_res = q1 + q2;
        q_res += q2;
        q_res = q_res - q1;
        q_res -= q2;
        q_res = -q_res;
        q_res *= -1;
        q_res = q_res * 4;
        q_res /= 2;
        q_res = q_res / 2;
        if (rtl::Quaternion<E>::distance(q_res, q2) > eps) {
            ASSERT_ANY_THROW(STRING_STREAM("\tToo large imprecision."));
        }
    }
}

template <typename E>
void quaternionOperations(unsigned int rep, E eps)
{
    std::cout<<"\nTesting advanced Quaternion operations:"<<std::endl;

    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<E> rnd_element(-1, 1);
    auto lambda_el_gen = [&generator, &rnd_element](){ return rnd_element(generator); };

    for(unsigned int i = 0; i < rep; i++)
    {
        rtl::Quaternion<E> q1 = rtl::Quaternion<E>::random(lambda_el_gen), q2 = rtl::Quaternion<E>::random(lambda_el_gen);
        q1 = q1 / q1.norm();
        if (std::abs(q1.normSquared() - 1.0) > eps)
        {
            ASSERT_ANY_THROW(STRING_STREAM("\tNorm imprecision."));
        }
        auto q2i = q2.inverted();
        auto q2s = q2 * q2i;
        if (std::abs(q2s.scalar() - 1.0) > eps || rtl::Vector3D<E>::distance(q2s.vector(), rtl::Vector3D<E>::zeros()) > eps)
        {
            ASSERT_ANY_THROW(STRING_STREAM("\tInversion imprecision."));
        }
        auto q1s = (q1 + q1.conjugated()) / 2.0;
        if (std::abs(q1s.scalar() - q1.scalar()) > eps || rtl::Vector3D<E>::distance(q1s.vector(), rtl::Vector3D<E>::zeros()) > eps)
        {
            ASSERT_ANY_THROW(STRING_STREAM("\tConjugate imprecision."));
        }
        auto q2n = q2;
        q2n.normalize();
        if (rtl::Quaternion<E>::distance(q2 / q2.norm(), q2n) > eps)
        {
            ASSERT_ANY_THROW(STRING_STREAM("\tNormalization imprecision."));
        }
        auto slerp = q1.normalized().slerp(q2.normalized(), 0.5);
        if (q1.normalized().data().angularDistance(q2.normalized().data()) - q1.normalized().data().angularDistance(slerp.data()) - slerp.data().angularDistance(q2.normalized().data()) > eps)
        {
            ASSERT_ANY_THROW(STRING_STREAM("\tSlerp imprecision."));
        }
    }
}


TEST(t_quaternion_tests, general_test) {

    size_t repeat = 10000;
    float err_eps_f = 0.00001f;
    double err_eps_d = 0.000000001;

    // Quaternion
    quaternionConstruction<float>(repeat, err_eps_f);
    quaternionConstruction<double>(repeat, err_eps_d);

    quaternionArithmetic<float>(repeat, err_eps_f);
    quaternionArithmetic<double>(repeat, err_eps_d);

    quaternionOperations<float>(repeat, err_eps_f);
    quaternionOperations<double>(repeat, err_eps_d);
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}