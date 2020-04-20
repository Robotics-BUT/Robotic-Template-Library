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
    const static double C_E = 2.71828182845904523536;           //!< Euler number
    const static double C_LOG2E = 1.44269504088896340736;       //!< \f$\log_2 e \f$
    const static double C_LOG10E = 0.434294481903251827651;     //!< \f$\log_{10} e \f$
    const static double C_LN2 = 0.693147180559945309417;        //!< \f$\ln 2 \f$
    const static double C_LN10 = 2.30258509299404568402;        //!< \f$\ln 10 \f$
    const static double C_PI = 3.14159265358979323846;          //!< \f$\pi \f$
    const static double C_PI_2 = 1.57079632679489661923;        //!< \f$\pi / 2 \f$
    const static double C_PI_4 = 0.785398163397448309616;       //!< \f$\pi / 4\f$
    const static double C_1_PI = 0.318309886183790671538;       //!< \f$ 1 / \pi \f$
    const static double C_2_PI = 0.636619772367581343076;       //!< \f$ 2 / \pi \f$
    const static double C_2_SQRTPI = 1.12837916709551257390;    //!< \f$ 2 / \sqrt{\pi} \f$
    const static double C_SQRT2 = 1.41421356237309504880;       //!< \f$ \sqrt{2} \f$
    const static double C_SQRT1_2 = 0.707106781186547524401;    //!< \f$ 1 / \sqrt{2} \f$

    const static float C_Ef = 2.71828182845904523536f;          //!< Euler number
    const static float C_LOG2Ef = 1.44269504088896340736f;      //!< \f$\log_2 e \f$
    const static float C_LOG10Ef = 0.434294481903251827651f;    //!< \f$\log_{10} e \f$
    const static float C_LN2f = 0.693147180559945309417f;       //!< \f$\ln 2 \f$
    const static float C_LN10f = 2.30258509299404568402f;       //!< \f$\ln 10 \f$
    const static float C_PIf = 3.14159265358979323846f;         //!< \f$\pi \f$
    const static float C_PI_2f = 1.57079632679489661923f;       //!< \f$\pi / 2 \f$
    const static float C_PI_4f = 0.785398163397448309616f;      //!< \f$\pi / 4\f$
    const static float C_1_PIf = 0.318309886183790671538f;      //!< \f$ 1 / \pi \f$
    const static float C_2_PIf = 0.636619772367581343076f;      //!< \f$ 2 / \pi \f$
    const static float C_2_SQRTPIf = 1.12837916709551257390f;   //!< \f$ 2 / \sqrt{\pi} \f$
    const static float C_SQRT2f = 1.41421356237309504880f;      //!< \f$ \sqrt{2} \f$
    const static float C_SQRT1_2f = 0.707106781186547524401f;   //!< \f$ 1 / \sqrt{2} \f$
}

#endif // ROBOTICTEMPLATELIBRARY_UTILITYCONSTANTS_H
