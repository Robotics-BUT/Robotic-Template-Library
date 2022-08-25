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

    /*!
     * Custom particle that implements mandatory methods and data types.
     * Mandatory Methods:
     *  - ParticleType(...) - value constructor
     *  - static ParticleType random()
     *  - void move(Action)
     *  - scoretype[float] belief(Measurement)
     *  - static Result evaluation()
     *
     * Mandatory Data Types:
     *  - Action
     *  - Measurement
     *  - Result
     *
     * @tparam T Data type of value represented by the particle
     */
    template <typename T>
    class SimpleParticle {

        using score_type = float;

    public:

        class Action {
        public:
            explicit Action(const T& val) : value_{val} {}
            [[nodiscard]] T value() const {return value_;}
        private:
            T value_;
        };

        class Measurement {
        public:
            explicit Measurement(const T& val) : value_{val} {}
            [[nodiscard]] T value() const {return value_;}
        private:
            T value_;
        };

        class Result {
        public:
            explicit Result(T mean, T std_dev) : mean_{mean}, std_dev_{std_dev} {}
            [[nodiscard]] T mean() const {return mean_;}
            [[nodiscard]] T std_dev() const {return std_dev_;}
        private:
            T mean_;
            T std_dev_;
        };


        /*!
         * Value constructor
         * @param val State value represented by particle
         */
        explicit SimpleParticle(T val) : value_{val} {}

        /*!
         * Generates particle with random inner state value
         * @return Random particle
         */
        static SimpleParticle random() {
            return SimpleParticle(get_uniform_random_value(-100.0f, 100.0f));
        }

        /*!
         * Move particle's inner state by given control input
         * @param action Control input applied on each particle
         */
        void move(const SimpleParticle::Action& action) {
            value_ += action.value();
        }

        /*!
         * Evaluates belief of given particle corresponds with measurement
         * @param measurement States of the system modeled by particle filter
         * @return Belief score of particle itself
         */
        [[nodiscard]] score_type belief(const SimpleParticle::Measurement& measurement) {
            return gauss(cost(measurement));
        }

        /*!
         * Estimate system state based on the N selected particles (it can be mean, median, most common val, etc.)
         * @param vec Vector of particles that are used to estimate state simulated by particle filter
         * @return Estimated state of modeled system
         */
        [[nodiscard]] static Result evaluation(const std::vector<SimpleParticle>& vec) {
            T sum = 0.0;
            std::for_each(vec.begin(), vec.end(), [&](auto particle){
                sum += particle.value_;
            });

            T mean = sum/vec.size();
            T square_diff_sum = 0.0;
            std::for_each(vec.begin(), vec.end(), [&](auto particle){
                square_diff_sum += std::pow(particle.value_ - mean, 2.0);
            });

            return Result(mean, std::sqrt(square_diff_sum/vec.size()));
        }

    private:

        [[nodiscard]] score_type cost(const SimpleParticle::Measurement& measurement) {
            return abs(value_ - measurement.value());
        }

        [[nodiscard]] score_type gauss(float x) const {
            constexpr float mean = 0;
            constexpr float std_dev = 10;
            constexpr float variance = std_dev * std_dev;
            constexpr float sqrt_2_pi = 2.5066;
            constexpr float a = (1 / (std_dev * sqrt_2_pi));
            return a * expf(-0.5f * powf(x - mean, 2) / variance);
        }

        static float get_uniform_random_value(float min, float max) {
            static std::random_device r;
            static auto engine = std::default_random_engine(r());
            std::uniform_real_distribution<T> distribution(min, max);
            return distribution(engine);
        }

        T value_;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_SIMPLEPARTICLE_H
