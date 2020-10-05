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
#include "rtl/tf/GeneralTf.h"
#include "rtl/tf/TfTree.h"
#include "rtl/tf/TfTreeNode.h"

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

//! Translation transformation to std::ostream operator.
/*!
 *
 * @tparam dim dimensionality of the transformation.
 * @tparam E ElementType of the transformation.
 * @param os output stream.
 * @param tr transformation to be printed.
 * @return reference to \p os.
 */
template<int dim, typename E>
std::ostream & operator<<( std::ostream & os, const rtl::TranslationND<dim, E> &tr)
{
    os << "t: " << tr.trVec();
    return os;
}

//! Rotation transformation to std::ostream operator.
/*!
 *
 * @tparam dim dimensionality of the transformation.
 * @tparam E ElementType of the transformation.
 * @param os output stream.
 * @param rot transformation to be printed.
 * @return reference to \p os.
 */
template<int dim, typename E>
std::ostream & operator<<( std::ostream & os, const rtl::RotationND<dim, E> &rot)
{
    os << "R: " << rot.rotMat();
    return os;
}

//! 3D rotation transformation to std::ostream operator.
/*!
 *
 * @tparam dim dimensionality of the transformation.
 * @tparam E ElementType of the transformation.
 * @param os output stream.
 * @param rot transformation to be printed.
 * @return reference to \p os.
 */
template<typename E>
std::ostream & operator<<( std::ostream & os, const rtl::RotationND<3, E> &rot)
{
    os << "R: " << rot.rotMat() << "   axix: " << rot.rotAxis() << "   angle: " << rot.rotAngle();
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
    os << tr.rot() << "   " << tr.tr();
    return os;
}

//! General transformation to std::ostream operator.
/*!
 *
 * @tparam T One transformation type possibly hold by the rtl::GeneralTf template. At least one is mandatory.
 * @tparam Tfs Other transformation types possibly hold by the rtl::GeneralTf template.
 * @param os output stream.
 * @param gtr transformation to be printed.
 * @return reference to \p os.
 */
template<typename T, typename... Tfs> // Additional type T ensures, that this overload is only considered, if there is at least one parameter in the rtl::GeneralTf<> template.
std::ostream & operator<<( std::ostream & os, const rtl::GeneralTf<T, Tfs...> &gtr)
{
    gtr.visit([&os](auto&& arg){os << arg;});
    return os;
}

//! Transformation tree node to std::ostream operator.
/*!
 *
 * @tparam K Type of the key used in the node.
 * @tparam T Type of the transformation used in the node.
 * @param os output stream.
 * @param node node to be printed.
 * @return reference to /p os.
 */
template<typename K, typename T>
std::ostream & operator<<( std::ostream & os, const rtl::TfTreeNode<K, T> &node)
{
    os << node.key() << "   " << node.tf();
    return os;
}

//! Transformation tree to std::ostream. Note this function generates multi-line output due to complexity of the data structure.
/*!
 *
 * @tparam K Type of keys used int the tree.
 * @tparam T Type of transformation used in the tree.
 * @param os output stream.
 * @param tree tree to be printed.
 * @return reference to /p os.
 */
template<typename K, typename T>
std::ostream & operator<<( std::ostream & os, const rtl::TfTree<K, T> &tree)
{
    auto print_with_childs = [](std::ostream & os, const typename rtl::TfTree<K, T>::NodeType& n) -> void
            {
                auto print_with_childs_impl=[](std::ostream & os, const typename rtl::TfTree<K, T>::NodeType& n, const auto& print_with_childs_ref) -> void
                        {
                            for (size_t i = 0; i < n.depth(); i++) os << "\t";
                            os << n << "\n";
                            for (const auto c : n.children())
                                print_with_childs_ref(os, *c, print_with_childs_ref);
                        };
                print_with_childs_impl(os, n, print_with_childs_impl);
            };

    auto root = tree.root();
    os << root.key() << "\n";
    for (auto c : root.children())
        print_with_childs(os, *c);
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
