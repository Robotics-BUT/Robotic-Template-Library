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

#ifndef ROBOTICTEMPLATELIBRARY_RANDOM_H
#define ROBOTICTEMPLATELIBRARY_RANDOM_H

#include <random>
#include <chrono>
#include <type_traits>

#include "rtl/Core.h"
#include "rtl/Transformation.h"

namespace rtl::test
{
    //! Class for comfortable generation of random values of any integer or floating point type.
    class Random
    {
    private:
        static auto& generator()
        {
            static auto generator = std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());
            return generator;
        }

    public:
        //! Provides a random value in given range with uniform distribution.
        /*!
         *
         * @tparam T type of the random value.
         * @param min lower bound of the range.
         * @param max upper bound of the range.
         * @return a random value.
         */
        template<typename T>
        static T uniformValue(T min, T max)
        {
            if constexpr (std::is_integral<T>())
            {
                std::uniform_int_distribution<T> dist(min, max);
                return dist(generator());
            }
            else if constexpr (std::is_floating_point<T>())
            {
                std::uniform_real_distribution<T> dist(min, max);
                return dist(generator());
            }
            else
                    static_assert(sizeof(T) != sizeof(T), "Unsupported type.");
        }

        //! Provides callable lambda which returns a random value in given range with uniform distribution on invocation.
        /*!
         *
         * @tparam T type of the random value.
         * @param min lower bound of the range.
         * @param max upper bound of the range.
         * @return a random value.
         */
        template<typename T>
        static auto uniformCallable(T min, T max)
        {
            if constexpr (std::is_integral<T>())
                return [min, max] () {std::uniform_int_distribution<T> dist(min, max); return dist(generator());};
            else if constexpr (std::is_floating_point<T>())
                return [min, max] () {std::uniform_real_distribution<T> dist(min, max); return dist(generator());};
            else
                    static_assert(sizeof(T) != sizeof(T), "Unsupported type.");
        }
    };
}

#endif //ROBOTICTEMPLATELIBRARY_RANDOM_H
