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

#ifndef ROBOTICTEMPLATELIBRARY_SIMPLEPARTICLE_H
#define ROBOTICTEMPLATELIBRARY_SIMPLEPARTICLE_H

#include <random>
#include <cmath>

namespace rtl {

    template <typename T>
    class SimpleParticle {
    public:

        class Result{
        public:
            Result(T mean, T std_dev) : mean_{mean}, std_dev_{std_dev} {}
            [[nodiscard]] T mean() const {return mean_;}
            [[nodiscard]] T std_dev() const {return std_dev_;}
        private:
            T mean_;
            T std_dev_;
        };

        explicit SimpleParticle(T val) : value_{val} {}

        static SimpleParticle random() {
            static std::random_device r;
            static auto engine = std::default_random_engine(r());
            static std::uniform_real_distribution<T> distribution(-100.0f, 100.0f);
            return SimpleParticle(distribution(engine));
        }

        void move(const SimpleParticle& action) {
            value_ += action.value_;
        }

        T belief(const SimpleParticle& measurement) {
            return gauss(cost(measurement));
        }

        [[nodiscard]] T value() const {return value_;}

        [[nodiscard]] static Result evaluation(const std::vector<SimpleParticle>& vec) {
            T sum = 0.0;
            std::for_each(vec.begin(), vec.end(), [&](auto particle){
                sum += particle.value();
            });

            T mean = sum/vec.size();
            T square_diff_sum = 0.0;
            std::for_each(vec.begin(), vec.end(), [&](auto particle){
                square_diff_sum += std::pow(particle.value() - mean, 2.0);
            });

            return Result(mean, std::sqrt(square_diff_sum/vec.size()));
        }

    private:

        [[nodiscard]] T cost(const SimpleParticle& measurement) {
            return abs(value_ - measurement.value_);
        }

        [[nodiscard]] T gauss(float x) const {
            constexpr T mean = 0;
            constexpr T std_dev = 10;
            constexpr T variance = std_dev * std_dev;
            constexpr T sqrt_2_pi = 2.5066;
            constexpr T a = (1 / (std_dev * sqrt_2_pi));
            return a * exp(-0.5 * std::pow(x - mean, 2) / variance);
        }

        T value_;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_SIMPLEPARTICLE_H
