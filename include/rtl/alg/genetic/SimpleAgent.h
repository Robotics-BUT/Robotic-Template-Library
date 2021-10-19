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

#ifndef ROBOTICTEMPLATELIBRARY_SIMPLEAGENT_H
#define ROBOTICTEMPLATELIBRARY_SIMPLEAGENT_H

namespace rtl {

    template <typename T>
    class SimpleAgent {
    public:

        explicit SimpleAgent(T val) : value_{val}{}

        static SimpleAgent random() {
            static std::random_device r;
            static auto engine = std::default_random_engine(r());
            static std::uniform_real_distribution<float> distribution(-100.0f, 100.0f);
            return SimpleAgent(static_cast<T>(distribution(engine)));
        }

        T value() const {
            return value_;
        }

        float score() {
            return 1.0f / (cost_(value_) + 0.001f);
        }

        SimpleAgent crossover(const SimpleAgent<T>& mate) {
            return SimpleAgent((value_ + mate.value_) / 2);
        }

        void mutate() {
            static std::random_device r;
            static auto engine = std::default_random_engine(r());
            static std::uniform_real_distribution<float> distribution(-1.0f, 1.0f);
            value_ += static_cast<T>(distribution(engine));
        }

        static void set_cost_fn(const std::function<float(T)>& fn) {
            cost_ = fn;
        }

    private:

        inline static std::function<float(T)> cost_ = [](const T& t){ return std::abs(0.0 - t);};
        T value_;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_SIMPLEAGENT_H
