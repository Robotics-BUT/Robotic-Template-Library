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

#ifndef ROBOTICTEMPLATELIBRARY_TYPEDETAILS_H
#define ROBOTICTEMPLATELIBRARY_TYPEDETAILS_H

#include <random>
#include <chrono>
#include <type_traits>

#include "rtl/Core.h"
#include "rtl/Transformation.h"

namespace rtl::test
{
    //! Type details for testing purposes.
    /*!
     * General template is not compilable and gives "No details specified for this type." static assert error message.
     * @tparam T examined type.
     */
    template<typename T>
    struct type
    {
        static_assert(sizeof(T) != sizeof(T), "No details specified for this type.");
    };

    //! Type details for float.
    template<>
    struct type<float>
    {
        //! Compile-time value of maximal allowed error in tests.
        static constexpr float allowedError() { return 0.001f;}
        static std::string description() { return "float"; }
    };

    //! Type details for double.
    template<>
    struct type<double>
    {
        //! Compile-time value of maximal allowed error in tests.
        static constexpr double allowedError() { return 0.000001; }
        //! Human readable name of given type returned as std::string.
        static std::string description() { return "double"; }
    };

    //! Type details for rtl::Matrix specializations.
    template<int r, int c, typename E>
    struct type<rtl::Matrix<r, c, E>>
    {
        //! Compile-time value of maximal allowed error in tests.
        static constexpr E allowedError() { return r * c * type<E>::allowedError(); }
        //! Human readable name of given type returned as std::string.
        static std::string description() { return "rtl::MatrixND<" + std::to_string(r) + ", " + std::to_string(c) + ", " + type<E>::description() + ">"; }
    };

    //! Type details for rtl::VectorND specializations.
    template<int d, typename E>
    struct type<rtl::VectorND<d, E>>
    {
        //! Compile-time value of maximal allowed error in tests.
        static constexpr E allowedError() {return d * type<E>::allowedError(); }
        //! Human readable name of given type returned as std::string.
        static std::string description() { return "rtl::VectorND<" + std::to_string(d) + ", " + type<E>::description() + ">"; }
    };

    //! Type details for rtl::RotationND specializations.
    template<int d, typename E>
    struct type<rtl::RotationND<d, E>>
    {
        //! Human readable name of given type returned as std::string.
        static std::string description() { return "rtl::RotationND<" + std::to_string(d) + ", " + type<E>::description() + ">"; }
    };

    //! Type details for rtl::TranslationND specializations.
    template<int d, typename E>
    struct type<rtl::TranslationND<d, E>>
    {
        //! Human readable name of given type returned as std::string.
        static std::string description() { return "rtl::TranslationND<" + std::to_string(d) + ", " + type<E>::description() + ">"; }
    };

    //! Type details for RigidTfND specializations.
    template<int d, typename E>
    struct type<rtl::RigidTfND<d, E>>
    {
        //! Human readable name of given type returned as std::string.
        static std::string description() { return "rtl::RigidTfND<" + std::to_string(d) + ", " + type<E>::description() + ">"; }
    };

}

#endif //ROBOTICTEMPLATELIBRARY_TYPEDETAILS_H
