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


#include <iostream>

#include <rtl/Core.h>
#include <rtl/Transformation.h>
#include <rtl/Test.h>

#define origin "origin"
#define key_1 "key_1"
#define key_2 "key_2"
#define key_3 "key_3"
#define key_4 "key_4"
#define key_5 "key_5"
#define key_6 "key_6"
#define key_7 "key_7"
#define key_8 "key_8"

double tf_distance(rtl::RigidTfND<3, double> tf1, rtl::RigidTfND<3, double>tf2) {
    auto vd = rtl::VectorND<3, double>::distance( tf1.trVec(), tf2.trVec());
    auto matd = rtl::Matrix<3, 3, double>::distance( tf1.rotMat(), tf2.rotMat());
    return vd + matd;
}

int main(int argc, char* argv[]) {

    auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);

    /// Single TF-type TfTree
    rtl::TfTree<std::string, rtl::RigidTfND<3,double>> tree{origin};

    auto tf_from_origin_to_1 = rtl::RigidTfND<3, double>::random(generator);
    auto tf_from_origin_to_2 = rtl::RigidTfND<3, double>::random(generator);
    auto tf_from_1_to_3 = rtl::RigidTfND<3, double>::random(generator);
    auto tf_from_1_to_4 = rtl::RigidTfND<3, double>::random(generator);
    auto tf_from_2_to_5 = rtl::RigidTfND<3, double>::random(generator);
    auto tf_from_2_to_6 = rtl::RigidTfND<3, double>::random(generator);
    auto tf_from_3_to_7 = rtl::RigidTfND<3, double>::random(generator);
    auto tf_from_5_to_8 = rtl::RigidTfND<3, double>::random(generator);

    ///
    ///                   origin
    ///               /            \
    ///             1               2
    ///           /   \           /   \
    ///         3       4       5       6
    ///       /                /
    ///     7                8
    ///

    tree.insert(key_1, tf_from_origin_to_1, origin);
    tree.insert(key_2, tf_from_origin_to_2, origin);

    tree.insert(key_3, tf_from_1_to_3, key_1);
    tree.insert(key_4, tf_from_1_to_4, key_1);

    tree.insert(key_5, tf_from_2_to_5, key_2);
    tree.insert(key_6, tf_from_2_to_6, key_2);

    tree.insert(key_7, tf_from_3_to_7, key_3);
    tree.insert(key_8, tf_from_5_to_8, key_5);

    auto chain_3_origin = tree.tf(key_3, origin);
    auto chain_7_8 = tree.tf(key_7, key_8);

    auto tf_7_8 = tf_from_5_to_8(tf_from_2_to_5(tf_from_origin_to_2(tf_from_origin_to_1.inverted()(tf_from_1_to_3.inverted()(tf_from_3_to_7.inverted())))));
    auto distance = tf_distance(chain_7_8.squash(), tf_7_8 );


    /// Mixed TF types

    using General3DTf = rtl::GeneralTf<rtl::RigidTfND<3,double>, rtl::TranslationND<3, double>, rtl::RotationND<3, double>>;
    rtl::TfTree<std::string, General3DTf> generalTree{origin};

    auto rigid = rtl::RigidTfND<3, double>::random(generator);
    auto rot = rtl::RotationND<3, double>::random(generator);
    auto trans = rtl::TranslationND<3, double>::random(generator);

    ///
    ///                   origin
    ///                  /      \
    ///               trans     rot
    ///                /          \
    ///              1             2
    ///             /
    ///          rigidTf
    ///           /
    ///         3
    ///

    generalTree.insert(key_1, trans, origin);
    generalTree.insert(key_2, rot, origin);
    generalTree.insert(key_3, rigid, key_1);

    auto chain_3_2 = generalTree.tf(key_3, key_2);

    auto tf_3_2 = rot(trans.inverted()(rigid.inverted()));
    distance = tf_distance(chain_3_2.squash(), tf_3_2 );

    return 0;
}