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

#ifndef ROBOTICTEMPLATELIBRARY_UTILITY_H
#define ROBOTICTEMPLATELIBRARY_UTILITY_H

/*! \file
 *  \brief Various utility functions of the RTL library not worthy an extra file.
 */

#include<cstdlib>
#include<string>

namespace rtl
{
    //! Fragile indication of debug mode compilation for debug only exceptions
    enum CompModes {DebugMode = 0,  //!< Debug mode indicatior.
            ReleaseMode             //!< Release mode indicator.
    };

#ifdef NDEBUG
    //! Returns actual compilation mode from rtl::CompModes.
    constexpr CompModes CompilationMode() { return rtl::ReleaseMode; }
#else
    //! Returns actual compilation mode from rtl::CompModes.
    constexpr CompModes CompilationMode() { return rtl::DebugMode; }
#endif

    //! std::string to numeric type conversion based on template type. Only specializations using std::stoXX() from standard library are callable.
    // We need T dependent expression in static_assert which is always false
    template <typename T> inline T ston(const std::string &str, size_t *idx = nullptr) { static_assert(sizeof(T) != sizeof(T), "rtl::ston<T>() - Unsupported type T."); }
    //! std::string to double specialization of the ston() template.
    template<> inline double ston(const std::string &str, size_t *idx) { return std::stod(str, idx); }
    //! std::string to float specialization of the ston() template.
    template<> inline float ston(const std::string &str, size_t *idx) { return std::stof(str, idx); }
    //! std::string to int specialization of the ston() template.
    template<> inline int ston(const std::string &str, size_t *idx) { return std::stoi(str, idx); }
    //! std::string to long int specialization of the ston() template.
    template<> inline long ston(const std::string &str, size_t *idx) { return std::stol(str, idx); }
    //! std::string to long double specialization of the ston() template.
    template<> inline long double ston(const std::string &str, size_t *idx) { return std::stold(str, idx); }
    //! std::string to long long int specialization of the ston() template.
    template<> inline long long ston(const std::string &str, size_t *idx) { return std::stoll(str, idx); }
    //! std::string to unsigned long int specialization of the ston() template.
    template<> inline unsigned long ston(const std::string &str, size_t *idx) { return std::stoul(str, idx); }
    //! std::string to unsigned long long int specialization of the ston() template.
    template<> inline unsigned long long ston(const std::string &str, size_t *idx) { return std::stoull(str, idx); }
}
#endif // ROBOTICTEMPLATELIBRARY_UTILITY_H
