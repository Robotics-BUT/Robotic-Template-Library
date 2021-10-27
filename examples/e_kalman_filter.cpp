// This file is part of the Robotic Template Library (RTL), a C++
// template library for usage in robotic research and applications
// under the MIT licence:
//
// Copyright 2021 Brno University of Technology
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
// Contact person: Adam Ligocki <adam.ligocki@vutbr.cz>

#include <rtl/Algorithms.h>
#include <rtl/Core.h>
#include <iostream>

float get_rand_norm_dist(float min, float max) {
    static std::random_device r;
    static auto engine = std::default_random_engine(r());
    std::normal_distribution<float> distribution(min, max);
    return distribution(engine);
}

int main() {

    float acc = 0.1f;
    float dt_step = 0.1f;

    float process_noise = 0.01;
    float observation_noise = 0.01;

    auto kf = rtl::Kalman<float, 2, 1, 1>(0.1, 0.1);

    auto A = rtl::Matrix<2, 2, float>::zeros();
    A.setRow(0, rtl::Vector2f{1.0f, dt_step});
    A.setRow(1, rtl::Vector2f{0.0f, 1});
    kf.set_transision_matrix(A);


    auto H = rtl::Matrix<1, 2, float>::zeros();
    H.setRow(0, rtl::Vector2f{1.0f, 0.0f});
    kf.set_measurement_matrix(H);


    auto B = rtl::Matrix<2,1,float>::zeros();
    B.setColumn(0, rtl::Vector2f{0.5f*dt_step*dt_step, dt_step});
    kf.set_control_matrix(B);


    auto Q = rtl::Matrix<2,2,float>::zeros();
    Q.setRow(0, rtl::Vector2f{powf(dt_step, 4) * process_noise, powf(dt_step, 3) * process_noise});
    Q.setRow(1, rtl::Vector2f{powf(dt_step, 3) * process_noise, powf(dt_step, 2) * process_noise});
    kf.set_process_noise_covariance_matrix(Q);

    auto R = rtl::Matrix<1,1,float>::zeros();
    R.setElement(0, 0, observation_noise);
    kf.set_measurement_noise_covariance_matrix(R);

    for (size_t i = 1 ; i <= 100 ; i++) {    // 10s

        auto time = i * dt_step;
        auto speed = acc * time;
        auto pose = 0.5f * acc * powf(time, 2);

        auto control = rtl::Matrix<1,1,float>::zeros();
        control.setElement(0,0, acc + get_rand_norm_dist(-process_noise, process_noise));
        kf.predict(control);

        auto measurement = rtl::Matrix<1, 1, float>::zeros();
        measurement.setElement(0,0, pose + get_rand_norm_dist(-observation_noise, observation_noise));
        kf.correct(measurement);

        std::cout << " - - - - - - - - - - " << std::endl;
        std::cout << " time: " << i * dt_step << std::endl;
        std::cout << " kf pose: " << kf.states().getElement(0, 0) << " gt: " << pose << std::endl;
        std::cout << " kf speed: " << kf.states().getElement(1, 0) << " gt: " << speed << std::endl;
    }

    return 0;
}