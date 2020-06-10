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

#ifndef ROBOTICTEMPLATELIBRARY_UTILITYCONSTANTS_H
#define ROBOTICTEMPLATELIBRARY_UTILITYCONSTANTS_H

/*! \file
 *  \brief Mathematical constants in rtl namespace. Use instead of M_... constants from \<cmath\> which are not mandatory in standard library implementations.
 */

namespace rtl
{
    template<typename T>
    const static T C_E = (T)2.71828182845904523536L;           //!< Euler number
    template<typename T>
    const static T C_LOG2E = (T)1.44269504088896340736L;       //!< \f$\log_2 e \f$
    template<typename T>
    const static T C_LOG10E = (T)0.434294481903251827651L;     //!< \f$\log_{10} e \f$
    template<typename T>
    const static T C_LN2 = (T)0.693147180559945309417L;        //!< \f$\ln 2 \f$
    template<typename T>
    const static T C_LN10 = (T)2.30258509299404568402L;        //!< \f$\ln 10 \f$
    template<typename T>
    const static T C_PI = (T)3.14159265358979323846L;          //!< \f$\pi \f$
    template<typename T>
    const static T C_PI_2 = (T)1.57079632679489661923L;        //!< \f$\pi / 2 \f$
    template<typename T>
    const static T C_PI_4 = (T)0.785398163397448309616L;       //!< \f$\pi / 4\f$
    template<typename T>
    const static T C_1_PI = (T)0.318309886183790671538L;       //!< \f$ 1 / \pi \f$
    template<typename T>
    const static T C_2_PI = (T)0.636619772367581343076L;       //!< \f$ 2 / \pi \f$
    template<typename T>
    const static T C_2_SQRTPI = (T)1.12837916709551257390L;    //!< \f$ 2 / \sqrt{\pi} \f$
    template<typename T>
    const static T C_SQRT2 = (T)1.41421356237309504880L;       //!< \f$ \sqrt{2} \f$
    template<typename T>
    const static T C_SQRT1_2 = (T)0.707106781186547524401L;    //!< \f$ 1 / \sqrt{2} \f$

    const static auto C_Ef = C_E<float>;                      //!< Euler number
    const static auto C_LOG2Ef = C_LOG2E<float>;              //!< \f$\log_2 e \f$
    const static auto C_LOG10Ef = C_LOG10E<float>;            //!< \f$\log_{10} e \f$
    const static auto C_LN2f = C_LN2<float>;                  //!< \f$\ln 2 \f$
    const static auto C_LN10f = C_LN10<float>;                //!< \f$\ln 10 \f$
    const static auto C_PIf = C_PI<float>;                    //!< \f$\pi \f$
    const static auto C_PI_2f = C_PI_2<float>;                //!< \f$\pi / 2 \f$
    const static auto C_PI_4f = C_PI_4<float>;                //!< \f$\pi / 4\f$
    const static auto C_1_PIf = C_1_PI<float>;                //!< \f$ 1 / \pi \f$
    const static auto C_2_PIf = C_2_PI<float>;                //!< \f$ 2 / \pi \f$
    const static auto C_2_SQRTPIf = C_2_SQRTPI<float>;        //!< \f$ 2 / \sqrt{\pi} \f$
    const static auto C_SQRT2f = C_SQRT2<float>;              //!< \f$ \sqrt{2} \f$
    const static auto C_SQRT1_2f = C_SQRT1_2<float>;          //!< \f$ 1 / \sqrt{2} \f$

    const static auto C_Ed = C_E<double>;                     //!< Euler number
    const static auto C_LOG2Ed = C_LOG2E<double>;             //!< \f$\log_2 e \f$
    const static auto C_LOG10Ed = C_LOG10E<double>;           //!< \f$\log_{10} e \f$
    const static auto C_LN2d = C_LN2<double>;                 //!< \f$\ln 2 \f$
    const static auto C_LN10d = C_LN10<double>;               //!< \f$\ln 10 \f$
    const static auto C_PId = C_PI<double>;                   //!< \f$\pi \f$
    const static auto C_PI_2d = C_PI_2<double>;               //!< \f$\pi / 2 \f$
    const static auto C_PI_4d = C_PI_4<double>;               //!< \f$\pi / 4\f$
    const static auto C_1_PId = C_1_PI<double>;               //!< \f$ 1 / \pi \f$
    const static auto C_2_PId = C_2_PI<double>;               //!< \f$ 2 / \pi \f$
    const static auto C_2_SQRTPId = C_2_SQRTPI<double>;       //!< \f$ 2 / \sqrt{\pi} \f$
    const static auto C_SQRT2d = C_SQRT2<double>;             //!< \f$ \sqrt{2} \f$
    const static auto C_SQRT1_2d = C_SQRT1_2<double>;         //!< \f$ 1 / \sqrt{2} \f$
}

#endif // ROBOTICTEMPLATELIBRARY_UTILITYCONSTANTS_H
