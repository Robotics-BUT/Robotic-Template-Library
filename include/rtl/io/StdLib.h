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

#ifndef ROBOTICTEMPLATELIBRARY_IO_STDLIB_H
#define ROBOTICTEMPLATELIBRARY_IO_STDLIB_H

#include <string>
#include <ostream>

#include "rtl/core/VectorND.h"
#include "rtl/core/LineSegmentND.h"
#include "rtl/tf/RigidTfND.h"

/*! \file
 *  \brief RTL export to STL streams.
 */

//! VectorND to std::ostream operator.
/*!
 *
 * @tparam dim dimensionality of the vector.
 * @tparam E ElementType of the vector.
 * @param os output stream.
 * @param v the vector to be printed.
 * @return reference to \p os.
 */
template <int dim, typename E>
std::ostream & operator<<( std::ostream & os, const rtl::VectorND<dim, E> &v)
{
    os << "[";
    int i = 0;
    while (true)
    {
        os << v[i];
        i++;
        if (i < dim)
            os << ", ";
        else
            break;
    }
    os << "]";
    return os;
}

//! Matrix to std::ostream operator.
/*!
 *
 * @tparam rows number of rows of the matrix.
 * @tparam cols number of columns of the matrix.
 * @tparam E ElementType of the matrix.
 * @param os output stream.
 * @param m the matrix to be printed.
 * @return reference to \p os.
 */
template <int rows, int cols, typename E>
std::ostream & operator<<( std::ostream & os, const rtl::Matrix<rows, cols, E> &m)
{
    os << "[";
    int r = 0, c = 0;
    while (true)
    {
        while (true)
        {
            os << m(r, c);
            c++;
            if (c < cols)
                os << ", ";
            else
                break;
        }
        c = 0;
        r++;
        if (r < rows)
            os << "; ";
        else
            break;
    }
    os << "]";
    return os;
}

//! Quaternion to std::ostream operator.
/*!
 *
 * @tparam E ElementType of the quaternion.
 * @param os output stream.
 * @param q quaternion  to be printed.
 * @return reference to \p os.
 */
template<typename E>
std::ostream & operator<<( std::ostream & os, const rtl::Quaternion<E> &q)
{
    os << q.w() << " ";
    os << ((q.x() >= 0) ? "+ " : "- ");
    os << std::abs(q.x()) << "i ";
    os << ((q.y() >= 0) ? "+ " : "- ");
    os << std::abs(q.y()) << "j ";
    os << ((q.z() >= 0) ? "+ " : "- ");
    os << std::abs(q.z()) << "k";
    return os;
}

//! Rigid transformation to std::ostream operator.
/*!
 *
 * @tparam dim dimensionality of the transformation.
 * @tparam E ElementType of the transformation.
 * @param os output stream.
 * @param tr transformation to be printed.
 * @return reference to \p os.
 */
template<int dim, typename E>
std::ostream & operator<<( std::ostream & os, const rtl::RigidTfND<dim, E> &tr)
{
    os << "R: " << tr.rotMat() << "   t: " << tr.trVec();
    return os;
}

//! Three dimensional rigid transformation to std::ostream operator.
/*!
 *
 * @tparam E ElementType of the transformation.
 * @param os output stream.
 * @param tr transformation to be printed.
 * @return reference to \p os.
 */
template<typename E>
std::ostream & operator<<( std::ostream & os, const rtl::RigidTfND<3, E> &tr)
{
    os << "R: " << tr.rotMat() << "   t: " << tr.trVec() << "   axix:" << tr.rotAxis() << "   angle:" << tr.rotAngle();
    return os;
}

//! Line segment to std::ostream operator.
/*!
 *
 * @tparam dim dimensionality of the line segment.
 * @tparam E ElementType of the line segment.
 * @param os output stream.
 * @param ls line segment to be printed.
 * @return reference to \p os.
 */
template <int dim, typename E>
std::ostream & operator<<( std::ostream & os, const rtl::LineSegmentND<dim, E> &ls)
{
    os << ls.beg() << "-->" << ls.end();
    return os;
}

namespace rtl::io
{
    // put formatted versions here
}

#endif //ROBOTICTEMPLATELIBRARY_IO_STDLIB_H
