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
#include <vector>

#include "rtl/Core.h"
#include "rtl/Transformation.h"

#include <eigen3/Eigen/Dense>

// Vector2D::angle() test
template <typename E>
void vector2DAngleCcwTest(float step, float epsilon)
{
    rtl::Vector2D<E> base(1, 0);
    float angle;

    std::cout << "\nVector2D::angle() test:" << std::endl;
    std::cout << "\tStep sum_nr: " << step << std::endl;
    std::cout << "\tEpsilon sum_nr: " << epsilon << std::endl;

    size_t err_cnt = 0;
    for (float i = -rtl::C_PIf + step; i <= rtl::C_PIf; i+=step)
    {
        angle = rtl::Vector2D<E>::angleCcw(base, rtl::Vector2D<E>(std::cos(i), std::sin(i)));
        if(std::abs(i - angle) > epsilon)
            err_cnt++;
    }
    std::cout << "\tPrecision errors: " << err_cnt << std::endl;
    std::cout << std::endl;
}


// Vector2D::angleFromZero() test
template <typename E>
void vector2DAngleFromZeroTest(float step, float epsilon)
{
    float angle;

    std::cout << "\nVector2D::angleFromZero() test:" << std::endl;
    std::cout << "\tStep sum_nr: " << step << std::endl;
    std::cout << "\tEpsilon sum_nr: " << epsilon << std::endl;

    size_t err_cnt = 0;
    for (E i = -rtl::C_PI + step; i <= rtl::C_PI; i+=step)
    {
        rtl::Vector2D<E> base = rtl::Vector2D<E>(std::cos(i), std::sin(i));
        angle = base.angleFromZero();
        if(std::abs(i - angle) > epsilon)
            err_cnt++;
    }
    std::cout << "\tPrecision errors: " << err_cnt << std::endl;
    std::cout << std::endl;
}

// Vector 2D angle function benchmarks
template <typename E>
void vector2DAngleSpeedTest(size_t repeat, float epsilon)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<float> coord(-1, 1);
    std::chrono::high_resolution_clock::time_point t_start, t_stop;
    float angle, angle_tot;

    std::cout << "\nOriented angle algorithm benchmarks:" << std::endl;

    std::vector<rtl::Vector2D<E>> v1, v2;
    for(size_t i = 0; i < repeat; i++)
    {
        v1.emplace_back(rtl::Vector2D<E>(coord(generator), coord(generator)));
        v2.emplace_back(rtl::Vector2D<E>(coord(generator), coord(generator)));
    }

    // AngleToZero timing
    angle_tot = 0.0f;
    t_start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < repeat; i++)
    {
        angle = std::atan2(v2[i].y(), v2[i].x()) - std::atan2(v1[i].y(), v1[i].x());
        if(angle > rtl::C_PI)
            angle -= 2 * rtl::C_PIf;
        else if(angle < -rtl::C_PI)
            angle += 2 * rtl::C_PIf;
        angle_tot += angle;
    }

    t_stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t_stop - t_start);
    std::cout << "\t2x AngleToZero time: " << time_span.count() <<std::endl;
    std::cout << "\t2x AngleToZero total: " << angle_tot <<std::endl;
    std::cout << "\tTruthfulness not tested since only atan2 and difference of angles is used." << std::endl;
    std::cout << std::endl;


    // Legacy Angle
    size_t nan_cnt = 0, err_cnt = 0;
    angle_tot = 0.0f;
    // Equivalency test:
    for (size_t i = 0; i < repeat; i++)
    {
        float angle1 = v2[i].angleFromZero() - v1[i].angleFromZero();
        if(angle1 > rtl::C_PI)
            angle1 -= 2 * rtl::C_PIf;
        else if(angle1 < -rtl::C_PI)
            angle1 += 2 * rtl::C_PIf;

        float cp, dp;
        float angle2;
        rtl::Vector2D<E> int_from = v1[i], int_to = v2[i];
        int_from.normalize();
        int_to.normalize();
        cp = rtl::Vector2D<E>::crossProduct(int_from, int_to);
        dp = rtl::Vector2D<E>::dotProduct(int_from, int_to);

        if (dp > 0)
            angle2 = std::asin(cp);
        else
            if(cp > 0)
                angle2 = rtl::C_PIf - std::asin(cp);
            else
                angle2 = - rtl::C_PIf - std::asin(cp);

        if(std::isnan(angle1) || std::isnan(angle2))
            nan_cnt++;

        if(std::abs(angle1 - angle2) > epsilon)
            err_cnt++;
    }

    // timing
    t_start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < repeat; i++)
    {
        float cp, dp;
        rtl::Vector2D<E> int_from = v1[i], int_to = v2[i];
        int_from.normalize();
        int_to.normalize();
        cp = rtl::Vector2D<E>::crossProduct(int_from, int_to);
        dp = rtl::Vector2D<E>::dotProduct(int_from, int_to);

        if (dp > 0)
            angle = std::asin(cp);
        else
            if(cp > 0)
                angle = rtl::C_PIf - std::asin(cp);
            else
                angle = - rtl::C_PIf - std::asin(cp);
        angle_tot += angle;
    }
    t_stop = std::chrono::high_resolution_clock::now();

    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t_stop - t_start);
    std::cout << "\tLegacy Angle time: " << time_span.count() <<std::endl;
    std::cout << "\tLegacy Angle total: " << angle_tot <<std::endl;
    std::cout << "\tProblem NaN: " <<nan_cnt<<std::endl;
    std::cout << "\tProblem Err: " <<err_cnt<<std::endl;
    std::cout << std::endl;


    nan_cnt = 0;
    err_cnt = 0;
    // Equivalency test:
    for (size_t i = 0; i < repeat; i++)
    {
        float angle1 = v2[i].angleFromZero() - v1[i].angleFromZero();
        if(angle1 > rtl::C_PI)
            angle1 -= 2 * rtl::C_PIf;
        else if(angle1 < -rtl::C_PI)
            angle1 += 2 * rtl::C_PIf;

        rtl::Vector2D<E> from_rot(-v1[i].y(), v1[i].x());
        float dot_orig = rtl::Vector2D<E>::dotProduct(v1[i], v2[i]);
        float dot_rot = rtl::Vector2D<E>::dotProduct(from_rot, v2[i]);
        float angle2 = std::atan2(dot_rot, dot_orig);

        if(std::isnan(angle1) || std::isnan(angle2))
            nan_cnt++;

        if(abs(angle1 - angle2) > epsilon)
            err_cnt++;
    }

    // Projection Angle timing
    angle_tot = 0.0f;

    t_start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < repeat; i++)
    {
        rtl::Vector2D<E> from_rot(-v1[i].y(), v1[i].x());
        float dot_orig = rtl::Vector2D<E>::dotProduct(v1[i], v2[i]);
        float dot_rot = rtl::Vector2D<E>::dotProduct(from_rot, v2[i]);
        angle_tot += std::atan2(dot_rot, dot_orig);
    }
    t_stop = std::chrono::high_resolution_clock::now();

    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t_stop - t_start);
    std::cout << "\tProjection Angle time: " << time_span.count() <<std::endl;
    std::cout << "\tProjection Angle total: " << angle_tot <<std::endl;
    std::cout << "\tProblem NaN: " <<nan_cnt<<std::endl;
    std::cout << "\tProblem Err: " <<err_cnt<<std::endl;
    std::cout << std::endl;
}

// VectorXX
void vectorXXConstruction()
{
    std::cout<<"\nVectorXX construction test:"<<std::endl;
    rtl::VectorND<3, int> v3i1, v3i2(v3i1), v3i3(v3i2.data());

    // Variadic template constructor test
    rtl::VectorND<2, float> v2f(2.0f,8.0f);
    std::cout << "\tVectorND<2, float> variadic: " << v2f[0] << "  " << v2f[1]  << std::endl;
    rtl::VectorND<3, float> v3f(2.0f,8.0f,8.0f);
    std::cout << "\tVectorND<3, float> variadic: " << v3f[0] << "  " << v3f[1] << "  " << v3f[2] << std::endl;
    rtl::VectorND<5, double> v5d(2.0, 8.0, 5.8, 6.3, 2.4);
    std::cout << "\tVectorND<5, double> variadic: " << v5d[0] << "  " << v5d[1] << "  " << v5d[2] << "  " << v5d[3] << "  " << v5d[4] << std::endl;

    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<float> rnd_float(-1, 1);
    auto el_rnd_gen = [&generator, &rnd_float](){ return rnd_float(generator); };

    auto v2fr = rtl::VectorND<2, float>::random(el_rnd_gen);
    std::cout << "\tVectorND<2, float>::random(): " << v2fr[0] << "  " << v2fr[1]  << std::endl;
    auto v3fr = rtl::VectorND<3, float>::random(el_rnd_gen);
    std::cout << "\tVectorND<3, float>::random(): " << v3fr[0] << "  " << v3fr[1] << "  " << v3fr[2] << std::endl;
    auto v5dr = rtl::VectorND<5, double>::random(el_rnd_gen);
    std::cout << "\tVectorND<5, double>::random(): " << v5dr[0] << "  " << v5dr[1] << "  " << v5dr[2] << "  " << v5dr[3] << "  " << v5dr[4] << std::endl;

    // Construction from Eigen
    auto ev3d = Eigen::Matrix<float, 3, 1>::Ones();
    rtl::Vector3f rtl_ev3d(ev3d);
    auto ev2d = Eigen::Matrix<float, 3, 1>::Ones();
    rtl::Vector3f rtl_ev2d(ev2d);
}

void vectorXXConversion()
{
    rtl::Vector2D<float> a_2d(1.0f,2.0f), b_2d;
    rtl::VectorND<2, float> a_nd2, b_nd2(3.0f, 3.0f);

    a_nd2 = a_2d;
    b_2d = b_nd2;

    std::cout << "\nConversion test:" << std::endl;
    std::cout << "\tVector2D -> VectorND<2, float>: " << a_nd2[0] << ", " << a_nd2[1] << std::endl;
    std::cout << "\tVectorND<2, float> -> Vector2D: " << b_2d[0] << ", " << b_2d[1] << std::endl;
}

void elementAccess()
{
    std::cout<<"\nElement acccess test:"<<std::endl;
    rtl::VectorND<3, double> v3i;
    rtl::VectorND<3, double>::ElementType x = 0.1, y = 0.2, z = 0.3;
    v3i.setElement(0, x);
    v3i.setElement(1, y);
    v3i.setElement(2, z);
    std::cout << "\tSet/get element: " << v3i.getElement(0) << "  " << v3i.getElement(1) << "  " << v3i.getElement(2) << std::endl;
    v3i[0] = v3i[1] = v3i[2] = 0.0;
    std::cout << "\tOperator []: " << v3i[0] << "  " << v3i[1] << "  " << v3i[2] << std::endl;
}

template <typename E>
void crossproductTest(unsigned int rep, float eps)
{
    std::cout << "\nCross-product test - equivalency of manual and Eigen implementation." << std::endl;
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<E> rnd_float(-1, 1);
    for(unsigned int i = 0; i < rep; i++)
    {
        rtl::Vector3D<E> v1(rnd_float(generator), rnd_float(generator), rnd_float(generator)), v2(rnd_float(generator), rnd_float(generator), rnd_float(generator));
        Eigen::Vector3f ev1, ev2;
        ev1[0] = v1[0];
        ev1[1] = v1[1];
        ev1[2] = v1[2];
        ev2[0] = v2[0];
        ev2[1] = v2[1];
        ev2[2] = v2[2];
        rtl::Vector3D<E> c_rtl = rtl::Vector3D<E>::crossProduct(v1, v2);
        Eigen::Vector3f c_eig = ev1.cross(ev2);
        float error = (c_rtl[0] - c_eig[0]) * (c_rtl[0] - c_eig[0]) + (c_rtl[1] - c_eig[1]) * (c_rtl[1] - c_eig[1]) + (c_rtl[2] - c_eig[2]) * (c_rtl[2] - c_eig[2]);
        if (error > eps)
            std::cout<<"\tExcessive error " << error << " detected" << std::endl;
    }
}

template<class V>
void vectorXXStaticOperations(unsigned int rep, float eps)
{
    std::cout<<"\nStatic operations test:"<<std::endl;
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<typename V::ElementType> rnd_element(-1, 1);
    auto el_rnd_gen = [&generator, &rnd_element](){ return rnd_element(generator); };

    std::cout << "\tNaN construction:" << std::endl;
    auto v_nan = V::nan();
    if (!v_nan.hasNaN())
        std::cout<<"\tNo NaNs in ::nan() initialized vector." << std::endl;

    std::cout<<"\tDistance computation:"<<std::endl;
    for(unsigned int i = 0; i < rep; i++)
    {
        auto v1 = V::random(el_rnd_gen);
        auto v2 = V::random(el_rnd_gen);
        typename V::DistanceType err = std::abs(V::distanceSquared(v1, v2) / V::distance(v1, v2) - (v1 - v2).length());
        if(err > eps)
            std::cout<<"\tExcessive error " << err << std::endl;
    }

    std::cout<<"\tScalar projection computation:"<<std::endl;
    for(unsigned int i = 0; i < rep; i++)
    {
        auto v1 = V::random(el_rnd_gen);
        auto v2 = V::random(el_rnd_gen);
        typename V::DistanceType err = std::abs(V::scalarProjection(v1, v2) - V::scalarProjectionOnUnit(v1, v2.normalized()));
        if(err > eps)
            std::cout<<"\tExcessive error " << err << std::endl;
    }

    std::cout<<"\tVector projection computation:"<<std::endl;
    for(unsigned int i = 0; i < rep; i++)
    {
        auto v1 = V::random(el_rnd_gen);
        auto v2 = V::random(el_rnd_gen);
        typename V::DistanceType err = (V::vectorProjection(v1, v2) - V::vectorProjectionOnUnit(v1, v2.normalized())).length();
        if(err > eps)
            std::cout<<"\tExcessive error " << err << std::endl;
    }
}

template<class V>
void normalization(unsigned int rep, typename V::DistanceType eps)
{
    std::cout<<"\nNormalization test:"<<std::endl;
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<typename V::ElementType> rnd_element(-1, 1);
    auto el_rnd_gen = [&generator, &rnd_element](){ return rnd_element(generator); };

    for(unsigned int i = 0; i < rep; i++)
    {
        auto v1 = V::random(el_rnd_gen);
        typename V::DistanceType err = std::abs(v1.normalized().length() - static_cast<typename V::DistanceType>(1));
        if(err > eps)
            std::cout<<"\tExcessive error " << err << std::endl;
    }
}

template<class V>
void transformation(unsigned int rep, typename V::DistanceType eps)
{
    std::cout<<"\nTransformation test:"<<std::endl;
    using TranformationType = typename std::conditional<V::dimensionality() == 2, rtl::Transformation2D<typename V::ElementType>,
            typename std::conditional<V::dimensionality() == 3, rtl::Transformation3D<typename V::ElementType>, void>::type>::type;
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<typename V::ElementType> rnd_element(-1, 1);
    std::uniform_real_distribution<typename V::ElementType> rnd_angle(-rtl::C_PI, rtl::C_PI);

    auto el_rnd_gen = [&generator, &rnd_element](){ return rnd_element(generator); };
    auto ang_rnd_gen = [&generator, &rnd_angle](){ return rnd_angle(generator); };

    for(unsigned int i = 0; i < rep; i++)
    {
        V v1 = V::random(el_rnd_gen);
        auto tr = TranformationType::random(ang_rnd_gen, el_rnd_gen);
        V v_tr = v1.transformed(tr);
        v_tr.transform(tr.inverted());
        typename V::DistanceType err = V::distance(v1, v_tr);
        if(err > eps)
            std::cout<<"\tExcessive error " << err << std::endl;
    }
}

int main()
{
    unsigned int repeat = 10000;
    float err_eps = 0.00001f;
    float angle_step = 0.01f;

    // VectorXX tests
    vectorXXConstruction();

    vectorXXConversion();

    elementAccess();

    vector2DAngleCcwTest<float>(angle_step, err_eps);
    vector2DAngleCcwTest<double>(angle_step, err_eps);

    vector2DAngleFromZeroTest<float>(angle_step, err_eps);
    vector2DAngleFromZeroTest<double>(angle_step, err_eps);

    vector2DAngleSpeedTest<float>(repeat, err_eps);
    vector2DAngleSpeedTest<double>(repeat, err_eps);

    crossproductTest<float>(repeat, err_eps);
    crossproductTest<double>(repeat, err_eps);

    vectorXXStaticOperations<rtl::Vector3f>(repeat, err_eps);
    vectorXXStaticOperations<rtl::Vector3d>(repeat, err_eps);
    vectorXXStaticOperations<rtl::Vector2f>(repeat, err_eps);
    vectorXXStaticOperations<rtl::Vector2d>(repeat, err_eps);
    vectorXXStaticOperations<rtl::VectorND<4, double>>(repeat, err_eps);

    normalization<rtl::Vector2f>(repeat, err_eps);
    normalization<rtl::Vector2d>(repeat, err_eps);
    normalization<rtl::Vector3f>(repeat, err_eps);
    normalization<rtl::Vector3d>(repeat, err_eps);
    normalization<rtl::VectorND<4, float>>(repeat, err_eps);

    transformation<rtl::Vector2f>(repeat, err_eps);
    transformation<rtl::Vector2d>(repeat, err_eps);
    transformation<rtl::VectorND<2, float>>(repeat, err_eps);
    transformation<rtl::VectorND<2, double>>(repeat, err_eps);

    transformation<rtl::Vector3f>(repeat, err_eps);
    transformation<rtl::Vector3d>(repeat, err_eps);
    transformation<rtl::VectorND<3, float>>(repeat, err_eps);
    transformation<rtl::VectorND<3, double>>(repeat, err_eps);

    return 0;
}

