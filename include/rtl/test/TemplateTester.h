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

#ifndef ROBOTICTEMPLATELIBRARY_TEMPLATETESTER_H
#define ROBOTICTEMPLATELIBRARY_TEMPLATETESTER_H

#include <random>
#include <chrono>
#include <type_traits>

#include "rtl/Core.h"
#include "rtl/Transformation.h"

namespace rtl::test
{
    //! Template for running tester objects with one typename template parameter.
    /*!
     * For usage with this template, the tester object should correspond to the following example:
     * @code
     *  template <typename T>
     *  struct Tester
     *  {
     *      static void testFunction(double i)
     *      {
     *          std::cout<<(T)i<<std::endl;
     *      }
     *  };
     *
     *  rtl::test::Types<Tester, int, double> t(3.14159265358979323846);
     * @endcode
     * which will produce the output:
     * @verbatim
       3.14159
       3
       @endverbatim
     * as the \p testFunction(double) of the \p Tester template is run for double and int specializations.
     *
     * @tparam Tester tester object template.
     * @tparam TestedTypes pack of types for which the tester object will be specialized.
     */
    template<template<typename> class Tester, typename ...TestedTypes>
    class Types
    {
    public:
        //! Constructor running all specializations of the \p Tester template for \p TestedTypes types with the arguments passed to it.
        /*!
         * To function properly, this constructor's argument pack \p TesterArgsTypes must correspond the testFunction(...) arguments of given \p Tester template.
         * @tparam TesterArgsTypes pack of argument types.
         * @param testerArgsValues pack of argument values.
         */
        template<typename... TesterArgsTypes>
        explicit Types(TesterArgsTypes... testerArgsValues)
        {
            typedef void(*TestFuncPtr)(TesterArgsTypes...);
            TestFuncPtr test_functions[func_nr];

            expandTypes<TestFuncPtr*, TestedTypes...>(test_functions);
            for (auto func : test_functions)
                (*func)(testerArgsValues...);
        }

    private:
        const static auto func_nr = sizeof...(TestedTypes);
        size_t func_counter = 0;

        template<typename TF, typename E, typename ...Es>
        typename std::enable_if<sizeof...(Es) != 0, void>::type expandTypes(TF test_functions)
        {
            expandTypes<TF, Es...>(test_functions);
            test_functions[func_counter++] = &Tester<E>::testFunction;
        }

        template<typename TF, typename E>
        void expandTypes(TF test_functions)
        {
            test_functions[func_counter++] = &Tester<E>::testFunction;
        }
    };

    //! Template for running tester objects with one integer and one typename template parameter.
    /*!
     * For usage with this template, the tester object should correspond to the following example:
     * @code
     *  template <int p, typename T>
     *  struct Tester
     *  {
     *      static void testFunction(double i)
     *      {
     *          std::cout<<(T)(p * i)<<std::endl;
     *      }
     *  };
     *
     *  rtl::test::RangeTypes<Tester, 1, 3, int, double> t(3.14159265358979323846);
     * @endcode
     * which will produce the output:
     * @verbatim
       3.14159
       6.28319
       9.42478
       3
       6
       9
       @endverbatim
     * as the \p testFunction(double) of the \p Tester template is run for double and int specializations with \p p in range [1,3].
     *
     * @tparam Tester tester object template.
     * @tparam r_min lower bound of the integer range.
     * @tparam r_max upper bound of the integer range.
     * @tparam TestedTypes pack of types for which the tester object will be specialized.
     */
    template<template<int, typename> class Tester, int r_min, int r_max, typename ...Types>
    class RangeTypes
    {
    public:
        //! Constructor running all specializations of the \p Tester template for \p TestedTypes types with the arguments passed to it.
        /*!
         * To function properly, this constructor's argument pack \p TesterArgsTypes must correspond the testFunction(...) arguments of given \p Tester template.
         * @tparam TesterArgsTypes pack of argument types.
         * @param testerArgsValues pack of argument values.
         */
        template<typename... TesterArgsTypes>
        explicit RangeTypes(TesterArgsTypes... testerArgsValues)
        {
            typedef void(*TestFuncPtr)(TesterArgsTypes...);
            TestFuncPtr test_functions[func_nr];

            expandTypes<TestFuncPtr*, r_max, Types...>(test_functions);
            for (auto func : test_functions)
                (*func)(testerArgsValues...);
        }

    private:
        const static auto func_nr = (r_max - r_min + 1) * sizeof...(Types);
        size_t func_counter = 0;

        template<typename TF, int r1, typename E, typename ...Es>
        typename std::enable_if<sizeof...(Es) != 0, void>::type expandTypes(TF test_functions)
        {
            expandTypes<TF, r1, Es...>(test_functions);
            expandDim1<TF, r1, E>(test_functions);
        }

        template<typename TF, int r1, typename E>
        void expandTypes(TF test_functions)
        {
            expandDim1<TF, r1, E>(test_functions);
        }

        template<typename TF, int r1, typename E>
        void expandDim1(TF test_functions)
        {
            if constexpr (r1 > r_min)
                expandDim1<TF, r1 - 1, E>(test_functions);
            test_functions[func_counter++] = &Tester<r1, E>::testFunction;
        }
    };

    //! Template for running tester objects with two integer and one typename template parameter.
    /*!
     * For usage with this template, the tester object should correspond to the following example:
     * @code
     *  template <int p, int q, typename T>
     *  struct Tester
     *  {
     *      static void testFunction(double i)
     *      {
     *          std::cout<<(T)(p * q * i)<<std::endl;
     *      }
     *  };
     *
     *  rtl::test::RangeRangeTypes<Tester, 1, 3, 1, 3, int, double> t(3.14159265358979323846);
     * @endcode
     * which will produce the output:
     * @verbatim
       3.14159
       6.28319
       9.42478
       6.28319
       12.5664
       18.8496
       9.42478
       18.8496
       28.2743
       3
       6
       9
       6
       12
       18
       9
       18
       28
       @endverbatim
     * as the \p testFunction(double) of the \p Tester template is run for double and int specializations with \p p in range [1,3].
     *
     * @tparam Tester tester object template.
     * @tparam r1_min lower bound of the first integer range.
     * @tparam r1_max upper bound of the first integer range.
     * @tparam r2_min lower bound of the second integer range.
     * @tparam r2_max upper bound of the second integer range.
     * @tparam TestedTypes pack of types for which the tester object will be specialized.
     */
    template<template<int, int, typename> class Tester, int r1_min, int r1_max, int r2_min, int r2_max, typename ...Types>
    class RangeRangeTypes
    {
    public:
        template<typename... TesterArgsTypes>
        explicit RangeRangeTypes(TesterArgsTypes... testerArgsValues)
        {
            typedef void(*TestFuncPtr)(TesterArgsTypes...);
            TestFuncPtr test_functions[func_nr];

            expandTypes<TestFuncPtr*, r1_max, r2_max, Types...>(test_functions);
            for (auto func : test_functions)
                (*func)(testerArgsValues...);
        }

    private:
        const static auto func_nr = (r1_max - r1_min + 1) * (r2_max - r2_min + 1) * sizeof...(Types);
        size_t func_counter = 0;

        template<typename TF, int r1, int r2, typename E, typename ...Es>
        typename std::enable_if<sizeof...(Es) != 0, void>::type expandTypes(TF test_functions)
        {
            expandTypes<TF, r1, r2, Es...>(test_functions);
            expandDim1<TF, r1, r2, E>(test_functions);
        }

        template<typename TF, int r1, int r2, typename E>
        void expandTypes(TF test_functions)
        {
            expandDim1<TF, r1, r2, E>(test_functions);
        }

        template<typename TF, int r1, int r2, typename E>
        void expandDim1(TF test_functions)
        {
            if constexpr (r1 > r1_min)
                expandDim1<TF, r1 - 1, r2, E>(test_functions);
            expandDim2<TF, r1, r2, E>(test_functions);
        }

        template<typename TF, int r1, int r2, typename E>
        void expandDim2(TF test_functions)
        {
            if constexpr (r2 > r2_min)
                expandDim2<TF, r1, r2 - 1, E>(test_functions);
            test_functions[func_counter++] = &Tester<r1, r2, E>::testFunction;
        }
    };
}

#endif //ROBOTICTEMPLATELIBRARY_TEMPLATETESTER_H
