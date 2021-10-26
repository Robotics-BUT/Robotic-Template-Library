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

    /*!
     * Simple implementation of agent used for GA (Genetic Algorithm).
     * Class implements the following mandatory methods and data types.
     *
     * Mandatory Methods:
     *  - Constructor
     *  - static AgentType random()
     *  - flaot score()
     *  - AgentType crossover(AgentType&)
     *  - mutate()
     *
     * one mandatory fit function:
     *  - std::function<float(AgentType)> fit_
     *
     * Mandatory data type:
     *  - Result
     *
     * and optional setters and getters
     *  - set_fit_fn(std::function<float(AgentType)>&)
     *  - T value()
     *
     * @tparam T dtype of Agents inner value
     * */
    template <typename T>
    class SimpleAgent {
    public:

        /*!
         * Value constructor
         * */
        explicit SimpleAgent(T val) : value_{val}{}

        /*!
         * Generates agent that represents random value from -100 to 100.
         * */
        static SimpleAgent random() {
            return SimpleAgent(static_cast<T>(uniform_random_val(-100.0f, 100.0f)));
        }

        /*!
         * Evaluates agent and gives his score.
         * */
        float score() {
            return fit_(value_);
        }

        /*!
         * Combines two agnets (parents) to a single offspring.
         * @param mate second parent
         * */
        SimpleAgent crossover(const SimpleAgent<T>& mate) {
            return SimpleAgent((value_ + mate.value_) / 2);
        }

        /*!
         * Agent is mutated randomly.
         * */
        void mutate() {
            value_ += static_cast<T>(uniform_random_val(-1.0f, 1.0f));
        }

        /*!
         * Sets new fit function. Can be changed between epochs.
         * @param fn pointer to new fit function
         * */
        static void set_fit_fn(const std::function<float(T)>& fn) {
            fit_ = fn;
        }

        /*!
         * Returns agent's inner value
         * */
        T value() const {
            return value_;
        }

    private:

        /*!
         * Return random value from <min, max> interval with uniform distribution
         * */
        static float uniform_random_val(float min, float max) {
            static std::random_device r;
            static auto engine = std::default_random_engine(r());
            std::uniform_real_distribution<float> distribution(min, max);
            return distribution(engine);
        }

        inline static std::function<float(T)> fit_ = [](const T&){ return 1.0f; };
        T value_;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_SIMPLEAGENT_H
