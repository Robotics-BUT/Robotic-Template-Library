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
#include <chrono>
#include <random>

#include "rtl/io/StdLib.h"
#include "rtl/Core.h"
#include "rtl/Transformation.h"
#include "rtl/Test.h"

template <typename T>
void printRndElTypes(int repeat, typename T::ElementType el_min = -1, typename T::ElementType el_max = 1)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<typename T::ElementType> rnd_element(el_min, el_max);
    auto lambda_el_gen = [&generator, &rnd_element](){ return rnd_element(generator); };

    for (int i = 0; i < repeat; i++)
        std::cout<<T::random(lambda_el_gen)<<std::endl;
}

template <typename T>
void printRndElRndAngTypes(int repeat, typename T::ElementType el_min = -1, typename T::ElementType el_max = 1)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<typename T::ElementType> rnd_element(el_min, el_max);
    std::uniform_real_distribution<typename T::ElementType> rnd_angle(-rtl::C_PI<typename T::ElementType>, rtl::C_PI<typename T::ElementType>);
    auto lambda_el_gen = [&generator, &rnd_element](){ return rnd_element(generator); };
    auto lambda_ang_gen = [&generator, &rnd_angle](){ return rnd_angle(generator); };

    for (int i = 0; i < repeat; i++)
        std::cout<<T::random(lambda_ang_gen, lambda_el_gen)<<std::endl;
}

template <typename E>
void runTestsForType(const std::string &type_name, int repeat, E el_min, E el_max)
{
    std::cout<<"\nPrinting rtl::VectorND<2, " << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::VectorND<2, E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rtl::VectorND<4, " << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::VectorND<4, E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rtl::Vector2D<" << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::Vector2D<E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rtl::Vector3D<" << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::Vector3D<E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rtl::Matrix<2, 2, " << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::Matrix<2, 2, E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rtl::Matrix<3, 2, " << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::Matrix<3, 2, E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rtl::Matrix<3, 3, " << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::Matrix<3, 3, E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rtl::Matrix<4, 1, " << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::Matrix<4, 1, E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rtl::Quaternion<" << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::Quaternion<E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rotation rtl::Quaternion<" << type_name << ">:"<<std::endl;
    printRndElRndAngTypes<rtl::Quaternion<E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rotation rtl::RigidTf2D<" << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::RigidTf2D<E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rotation rtl::RigidTf3D<" << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::RigidTf3D<E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rtl::LineSegmentND<2, " << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::LineSegmentND<2, E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rtl::LineSegmentND<4, " << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::LineSegmentND<4, E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rtl::LineSegment2D<" << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::LineSegment2D<E>>(repeat, el_min, el_max);
    std::cout<<"\nPrinting rtl::LineSegment3D<" << type_name << ">:"<<std::endl;
    printRndElTypes<rtl::LineSegment3D<E>>(repeat, el_min, el_max);
}

template<template<int, typename> class TfTemplate, int r_min, int r_max, typename... Es>
struct TesterWrapperGeneralTf
{
    template<int dim, typename E>
    struct TesterGeneralTf
    {
        static void testFunction(int rep)
        {
            auto el_gen = rtl::test::Random::uniformCallable((E)-1, (E)1);
            using T = TfTemplate<dim, E>;
            using GT = rtl::GeneralTf<T>;
            std::cout<<"\nPrinting " << rtl::test::type<GT>::description() << ":"<<std::endl;

            for (int i = 0; i < rep; i++)
            {
                GT gtf(T::random(el_gen));
                std::cout<<gtf<<std::endl;
            }
        }
    };

    explicit TesterWrapperGeneralTf(int rep)
    {
        rtl::test::RangeTypes<TesterGeneralTf, r_min, r_max, Es...> t(rep);
    }

};

int main()
{
    int repeat = 5;

    std::cout << "Small elements test" << std::endl;
    runTestsForType<float>("float", repeat, -10.0f, 10.0f);
    runTestsForType<double>("double", repeat, -10.0, 10.0);

    std::cout << "\n\n\n\nLarge elements test" << std::endl;
    runTestsForType<float>("float", repeat, -1e30f, 1e30f);
    runTestsForType<double>("double", repeat, -1e50, 1e50);

    TesterWrapperGeneralTf<rtl::TranslationND, 1, 5, float, double> t_gtf_tr(repeat);
    TesterWrapperGeneralTf<rtl::RotationND, 2, 5, float, double> t_gtf_rot(repeat);
    TesterWrapperGeneralTf<rtl::RigidTfND, 2, 5, float, double> t_gtf_rig(repeat);

    return 0;
}