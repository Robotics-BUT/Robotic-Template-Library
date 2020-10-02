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

#include <gtest/gtest.h>
#include <rtl/Transformation.h>
#include <rtl/Test.h>

#include "tf_test/key_generator.h"
#include "tf_test/tf_comparison.h"



template<int N, typename dtype, typename K>
struct TestInit {
    static void testFunction() {

        rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>> node{};
        ASSERT_EQ(node.depth(), 0);
    }
};


TEST(t_tf_tree_node, init) {
    [[maybe_unused]]auto keyGenTest = rtl::test::RangeTypesTypes<TestInit, RANGE_AND_DTYPES>::with<TYPES>{};
}



template<int N, typename dtype, typename K>
struct TestConstructors {
    static void testFunction() {

        auto keys = KeysGenerator<K>{keyN}.generateKyes();

        rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>> node{};
        ASSERT_EQ(node.depth(), 0);
        ASSERT_EQ(node.children().size() , 0);
        ASSERT_EQ(node.parent(), nullptr);


        rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>> node2{origin};
        ASSERT_EQ(node2.depth(), 0);
        ASSERT_EQ(node2.children().size() , 0);
        ASSERT_EQ(node2.key(), origin);
        ASSERT_EQ(node2.parent()->key(), node2.key());


        rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>> node_cp{origin};
        ASSERT_EQ(node_cp.depth(), 0);
        ASSERT_EQ(node_cp.children().size() , 0);
        ASSERT_EQ(node_cp.key(), origin);
        ASSERT_EQ(node_cp.parent()->key(), node_cp.key());


        rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>> node_mv{rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>>{origin}};
        ASSERT_EQ(node_mv.depth(), 0);
        ASSERT_EQ(node_mv.children().size() , 0);
        ASSERT_EQ(node_mv.key(), origin);
        ASSERT_EQ(node_mv.parent()->key(), node_mv.key());


        rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>> node3{rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>>{
            key_1,
            rtl::RigidTfND<N, dtype>::identity(),
            node}
        };
        ASSERT_EQ(node3.depth(), 1);
        ASSERT_EQ(node3.children().size() , 0);
        ASSERT_EQ(node.children().size() , 1);
        ASSERT_EQ(node3.key(), key_1);
        ASSERT_EQ(node3.parent()->key(), node.key());
    }
};


TEST(t_tf_tree_node, constructors) {
    [[maybe_unused]]auto keyGenTest = rtl::test::RangeTypesTypes<TestConstructors, RANGE_AND_DTYPES>::with<TYPES>{};
}



template<int N, typename dtype, typename K>
struct TestInitKey {
    static void testFunction() {

        auto keys = KeysGenerator<K>{keyN}.generateKyes();

        rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>> node(origin);
        ASSERT_EQ(node.key(), origin);
        ASSERT_EQ(node.depth(), 0);
        ASSERT_EQ(node.children().size() , 0);
        ASSERT_EQ(node.parent()->key(), node.key());
    }
};

TEST(t_tf_tree_node, init_key) {

    [[maybe_unused]]auto keyGenTest = rtl::test::RangeTypesTypes<TestInitKey, RANGE_AND_DTYPES>::with<TYPES>{};
}



template<int N, typename dtype, typename K>
struct TestNesting {
    static void testFunction() {

        auto keys = KeysGenerator<K>{keyN}.generateKyes();

        rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>> node1(key_1);
        rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>> node2(key_2, rtl::RigidTf3d::identity(), node1);
        rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>> node3(key_3, rtl::RigidTf3d::identity(), node2);

        ASSERT_EQ(node1.key(), key_1);
        ASSERT_EQ(node1.depth(), 0);
        ASSERT_EQ(node1.children().size() , 1);
        ASSERT_EQ((*node1.children().begin())->key() , key_2);
        ASSERT_EQ(node1.parent()->key(), node1.key());

        ASSERT_EQ(node2.key(), key_2);
        ASSERT_EQ(node2.depth(), 1);
        ASSERT_EQ(node2.children().size() , 1);
        ASSERT_EQ((*node2.children().begin())->key() , key_3);
        ASSERT_EQ(node2.parent()->key(), node1.key());

        ASSERT_EQ(node3.key(), key_3);

        ASSERT_EQ(node3.depth(), 2);
        ASSERT_EQ(node3.children().size() , 0);
        ASSERT_EQ(node3.parent()->key(), node2.key());
    }
};

TEST(t_tf_tree_node, node_nesting) {

    [[maybe_unused]]auto keyGenTest = rtl::test::RangeTypesTypes<TestInitKey, RANGE_AND_DTYPES>::with<TYPES>{};
}



template<int N, typename dtype, typename K>
struct TestTf {
    static void testFunction() {

        auto keys = KeysGenerator<K>{keyN}.generateKyes();

        auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);
        auto tf12 = rtl::RigidTfND<N, dtype>::random(generator);
        auto tf23 = rtl::RigidTfND<N, dtype>::random(generator);

        rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>> node1{rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>>{origin}};
        rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>> node2{key_1, tf12, node1};
        rtl::TfTreeNode<K, rtl::RigidTfND<N, dtype>> node3{key_2, tf23, node2};

        ASSERT_EQ(CompareTfsEqual(tf12, node2.tf()), true);
        ASSERT_EQ(CompareTfsEqual(tf23, node3.tf()), true);
    }
};

TEST(t_tf_tree_node, tf) {

        [[maybe_unused]]auto tfTests = rtl::test::RangeTypesTypes<TestTf, RANGE_AND_DTYPES>::with<TYPES>{};
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
