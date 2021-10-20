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

#include <gtest/gtest.h>

#include "rtl/Algorithms.h"


TEST(t_particle_filter, init) {
    auto particle_filter = rtl::ParticleFilter<rtl::SimpleParticle<float>, 10, 5>();
}


TEST(t_particle_filter, test_1) {

    auto particle_filter = rtl::ParticleFilter<rtl::SimpleParticle<float>, 100, 30>();
    for (size_t i = 0 ; i < 100 ; i++) {
        particle_filter.iteration(rtl::SimpleParticle<float>::Action(0.0f),
                                  rtl::SimpleParticle<float>::Measurement(0.0f));
    }

    auto result = particle_filter.evaluate();
    std::cout << "gt: " << 0.0 << " mean_pose: " << result.mean() << " std_dev_pose: " << result.std_dev() << std::endl;
    EXPECT_NEAR(result.mean(), 0.0, 5.0);
}



TEST(t_particle_filter, test_2) {

    auto particle_filter = rtl::ParticleFilter<rtl::SimpleParticle<float>, 1000, 300>();
    float step = 0.1;
    float measurement = 0.0;

    for (size_t i = 0 ; i < 100 ; i++) {
        measurement += step;
        particle_filter.iteration(rtl::SimpleParticle<float>::Action(step), rtl::SimpleParticle<float>::Measurement(measurement));
    }

    auto result = particle_filter.evaluate();
    std::cout << "gt: " << measurement << " mean_pose: " << result.mean() << " std_dev_pose: " << result.std_dev() << std::endl;
    EXPECT_NEAR(result.mean(), measurement, 5.0);
}


TEST(t_particle_filter, test_3) {

    auto particle_filter = rtl::ParticleFilter<rtl::SimpleParticle<float>, 10000, 3000>();
    float step = 0.1;
    float measurement = 0.0;

    for (size_t i = 0 ; i < 100 ; i++) {
        measurement += step;
        particle_filter.iteration(rtl::SimpleParticle<float>::Action(step), rtl::SimpleParticle<float>::Measurement(measurement));
    }

    auto result = particle_filter.evaluate();
    std::cout << "gt: " << measurement << " mean_pose: " << result.mean() << " std_dev_pose: " << result.std_dev() << std::endl;
    EXPECT_NEAR(result.mean(), measurement, 5.0);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
