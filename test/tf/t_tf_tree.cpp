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

#include <vector>
#include <typeinfo>
#include <iostream>
#include <chrono>
#include "tf_test/key_generator.h"
#include "tf_test/tf_comparison.h"
#include "rtl/io/StdLib.h"


TEST(t_tf_tree, key_generator) {
    rtl::test::Types<TestKeyGenerator, TYPES> keyGenTest(static_cast<size_t>(10));
}


//////////////////////////
/// Tests
//////////////////////////


template<int N, typename dtype, typename T>
struct TestInit {
    static void testFunction() {
        auto keyGen = KeysGenerator<T>{keyN};
        auto keys = keyGen.generateKyes();

        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree(origin);
        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), 1);

        tree.clear();

        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), 1);
    }
};


TEST(t_tf_tree, init) {
    [[maybe_unused]]auto keyGenTest = rtl::test::RangeTypesTypes<TestInit, 2, 4, float, double>::with<TYPES>{};
}


template<int N, typename dtype, typename T>
struct TestConstructors {
    static void testFunction() {
        auto keyGen = KeysGenerator<T>{keyN};
        auto keys = keyGen.generateKyes();

        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree(origin);
        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), 1);

        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree_copy(origin);
        ASSERT_EQ(tree_copy.empty(), false);
        ASSERT_EQ(tree_copy.size(), 1);
        auto tree_copy_root_address = &tree_copy.root();
        ASSERT_TRUE(&tree.root() != tree_copy_root_address);

        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree_move(rtl::TfTree<T, rtl::RigidTfND<N, dtype>>{origin});
        ASSERT_EQ(tree_move.empty(), false);
        ASSERT_EQ(tree_move.size(), 1);
    }
};

TEST(t_tf_tree, constructors) {
    [[maybe_unused]]auto constructorTests = rtl::test::RangeTypesTypes<TestConstructors, RANGE_AND_DTYPES>::with<TYPES>{};
}


template<int N, typename dtype, typename T>
void fill_tree_insert(rtl::TfTree<T, rtl::RigidTfND<N, dtype>>& tree, std::vector<T> keys) {
    auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);

    for (size_t i = 1 ; i < keys.size() ; i++) {
        auto tf = rtl::RigidTfND<N, dtype>::random(generator);
        tree.insert(keys.at(i), tf, keys.at(i-1));
        auto tf2 = tree.at(keys.at(i)).tf();

        bool res = CompareTfsEqual<N, dtype>(tf, tf2);
        ASSERT_EQ(res, true);
    }
}

template<int N, typename dtype, typename T>
struct TestInsert {
    static void testFunction() {
        auto keyGen = KeysGenerator<T>{keyN};
        auto keys = keyGen.generateKyes();

        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree(origin);

        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), 1);

        fill_tree_insert(tree, keys);

        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), keys.size());
    }
};

TEST(t_tf_tree, insert) {

    [[maybe_unused]]auto insertTest = rtl::test::RangeTypesTypes<TestInsert, RANGE_AND_DTYPES>::with<TYPES>{};
}


template<int N, typename dtype, typename T>
struct TestClear {
    static void testFunction() {
        auto keyGen = KeysGenerator<T>{keyN};
        auto keys = keyGen.generateKyes();

        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree(origin);

        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), 1);

        fill_tree_insert(tree, keys);
        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), keys.size());

        tree.clear();       // root should stay

        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), 1);

        fill_tree_insert(tree, keys);
        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), keys.size());
    }
};

TEST(t_tf_tree, clear) {

    [[maybe_unused]]auto clearTest = rtl::test::RangeTypesTypes<TestClear, RANGE_AND_DTYPES>::with<TYPES>{};
}


template<int N, typename dtype, typename T>
struct TestContains {
    static void testFunction() {
        auto keyGen = KeysGenerator<T>{keyN};
        auto keys = keyGen.generateKyes();

        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree(origin);

        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), 1);
        for (size_t i = 1 ; i < keys.size() ; i++) {
            ASSERT_EQ(tree.contains(keys.at(i)), false);
        }

        fill_tree_insert(tree, keys);
        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), keys.size());

        for (const auto& key : keys) {
            ASSERT_EQ(tree.contains(key), true);
        }
    }
};

TEST(t_tf_tree, contains) {

    [[maybe_unused]]auto containsTest = rtl::test::RangeTypesTypes<TestContains, RANGE_AND_DTYPES>::with<TYPES>{};
}

template<int N, typename dtype, typename T>
struct TestErase {
    static void testFunction() {
        auto keyGen = KeysGenerator<T>{keyN};
        auto keys = keyGen.generateKyes();
        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree(origin);

        for (typename std::vector<T>::iterator it = keys.end() - 1; it > keys.begin(); it--) {
            tree.erase(*it);
        }

        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), 1);
        fill_tree_insert(tree, keys);

        {
            size_t i = 0;
            for (typename std::vector<T>::iterator it = keys.end() - 1; it > keys.begin(); it--) {
                ASSERT_EQ(tree.empty(), false);
                ASSERT_EQ(tree.size(), keys.size() - i);
                tree.erase(*it);

                for (typename std::vector<T>::iterator it2 = (keys.end() - 1); it2 > keys.begin(); it2--) {

                    if (it2 >= it){
                        ASSERT_EQ(tree.contains(*it2), false);
                    } else {
                        ASSERT_EQ(tree.contains(*it2), true);
                    }
                }
                i++;
            }
        }
    }
};

TEST(t_tf_tree, erase) {
    [[maybe_unused]]auto eraseTest = rtl::test::RangeTypesTypes<TestErase, RANGE_AND_DTYPES>::with<TYPES>{};
}

template<int N, typename dtype, typename T>
struct TestErase2 {
    static void testFunction() {
        auto keyGen = KeysGenerator<T>{keyN};
        auto keys = keyGen.generateKyes();
        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree(origin);

        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), 1);

        fill_tree_insert(tree, keys);

        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), keys.size());

        tree.erase(keys.at(1));
        for (size_t i = 0 ; i < keys.size() ; i++) {
            if (i > 0) {
                ASSERT_EQ(tree.contains(keys.at(i)), false);
            } else {
                ASSERT_EQ(tree.contains(keys.at(i)), true);
            }
        }
    }
};


TEST(t_tf_tree, erase_2) {

    [[maybe_unused]]auto erase2Test = rtl::test::RangeTypesTypes<TestErase2, RANGE_AND_DTYPES>::with<TYPES>{};
}

template<int N, typename dtype, typename T>
struct RootTest {
    static void testFunction() {
        auto keyGen = KeysGenerator<T>{keyN};
        auto keys = keyGen.generateKyes();
        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree(origin);

        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), 1);

        ASSERT_EQ(tree.root().key(), origin);
        ASSERT_EQ(tree.root().children().size(), 0);
        ASSERT_EQ(tree.root().depth(), 0);
        ASSERT_EQ(tree.root().key() , tree.root().parent()->key());
    }
};

TEST(t_tf_tree, root) {

    [[maybe_unused]]auto rootTest = rtl::test::RangeTypesTypes<RootTest, RANGE_AND_DTYPES>::with<TYPES>{};
}


template<int N, typename dtype, typename T>
struct AtTest {
    static void testFunction() {
        auto keyGen = KeysGenerator<T>{keyN};
        auto keys = keyGen.generateKyes();
        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree(origin);

        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), 1);

        fill_tree_insert(tree, keys);

        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), keys.size());

        for (const auto& key : keys) {
            ASSERT_EQ(tree.at(key).key(), key);
            ASSERT_EQ(tree[key].key(), key);
            ASSERT_EQ( &tree[key], &tree.at(key));
        }
    }
};

TEST(t_tf_tree, at) {

    [[maybe_unused]]auto atTest = rtl::test::RangeTypesTypes<AtTest, RANGE_AND_DTYPES>::with<TYPES>{};
}

template<int N, typename dtype, typename T>
struct TreeStructureTest {
    static void testFunction() {
        auto keyGen = KeysGenerator<T>{keyN};
        auto keys = keyGen.generateKyes();
        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree(origin);

        fill_tree_insert(tree, keys);

        for (size_t i = 0 ; i < keys.size() ; i++) {
            auto& node1 = tree.at(keys.at(i));
            auto& node2 = tree[keys.at(i)];
            ASSERT_EQ( &node1, &node2);
            ASSERT_EQ( node1.depth(), i);

            if(i > 0) {
                const auto& parent = tree[keys.at(i-1)];
                auto node_parent = node1.parent();
                ASSERT_EQ( node_parent, &parent);
            }
        }
    }
};

TEST(t_tf_tree, tree_structure) {
    [[maybe_unused]]auto treeStructureTest = rtl::test::RangeTypesTypes<TreeStructureTest, RANGE_AND_DTYPES>::with<TYPES>{};
}


template<int N, typename dtype, typename T>
struct TestTfFromTo {
    static void testFunction() {
        auto keyGen = KeysGenerator<T>{keyN};
        auto keys = keyGen.generateKyes();

        auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);
        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree(origin);

        auto tf1 = rtl::RigidTfND<N, dtype>::random(generator);
        auto tf2 = rtl::RigidTfND<N, dtype>::random(generator);
        auto tf3 = rtl::RigidTfND<N, dtype>::random(generator);
        auto tf4 = rtl::RigidTfND<N, dtype>::random(generator);
        auto tf5 = rtl::RigidTfND<N, dtype>::random(generator);
        auto tf6 = rtl::RigidTfND<N, dtype>::random(generator);
        auto tf7 = rtl::RigidTfND<N, dtype>::random(generator);
        auto tf8 = rtl::RigidTfND<N, dtype>::random(generator);
        auto tf9 = rtl::RigidTfND<N, dtype>::random(generator);

        /*
         *
         *          / tf2  -   tf5   -   tf9
         *         /       \   tf6
         * origin
         *         \
         *          \ tf1  -   tf3   -   tf7
         *                 \          \  tf8
         *                  \  tf4
         */

        tree.insert(key_1, tf1, origin);
        tree.insert(key_2, tf2, origin);

        tree.insert(key_3, tf3, key_1);
        tree.insert(key_4, tf4, key_1);

        tree.insert(key_5, tf5, key_2);
        tree.insert(key_6, tf6, key_2);

        tree.insert(key_7, tf7, key_3);
        tree.insert(key_8, tf8, key_3);
        tree.insert(key_9, tf9, key_5);


        auto identity = rtl::RigidTfND<N, dtype>::identity();
        auto tfChain = tree.tf(key_8, key_9);

        std::cout << tf1 << std::endl;
        auto cumulated = tfChain(identity);
        auto cumulated2 = tf9(tf5(tf2(tf1.inverted()(tf3.inverted()(tf8.inverted())))));
        ASSERT_EQ(CompareTfsEqual(cumulated, cumulated2), true);
    }
};

TEST(t_tf_tree, tree_tf_from_to) {
    [[maybe_unused]]auto tfFromToTest = rtl::test::RangeTypesTypes<TestTfFromTo, RANGE_AND_DTYPES>::with<TYPES>{};
}


template<int N, typename dtype, typename T>
struct APITest {
    static void testFunction() {

        auto keyGen = KeysGenerator<T>{keyN};
        auto keys = keyGen.generateKyes();

        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree(origin);
        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree_copy(tree);
        rtl::TfTree<T, rtl::RigidTfND<N, dtype>> tree_move(rtl::TfTree<T, rtl::RigidTfND<N, dtype>>{origin});


        ASSERT_EQ(tree.empty(), false);
        ASSERT_EQ(tree.size(), 1);

        ASSERT_EQ(tree_copy.empty(), false);
        ASSERT_EQ(tree_copy.size(), 1);

        ASSERT_EQ(tree_move.empty(), false);
        ASSERT_EQ(tree_move.size(), 1);

        tree_copy.clear();
        tree_move.clear();

        ASSERT_EQ(tree_copy.empty(), false);
        ASSERT_EQ(tree_copy.size(), 1);

        ASSERT_EQ(tree_move.empty(), false);
        ASSERT_EQ(tree_move.size(), 1);

        tree_copy = tree;
        tree_move = rtl::TfTree<T, rtl::RigidTfND<N, dtype>>(origin);

        ASSERT_EQ(tree.contains(key_1), false);
        tree.insert(key_1, rtl::RigidTfND<N, dtype>::identity(), origin);
        ASSERT_EQ(tree.contains(key_1), true);

        tree.erase(key_1);
        ASSERT_EQ(tree.contains(key_1), false);
        ASSERT_EQ(tree.root().key(), origin);

        auto new_tf = rtl::RigidTfND<N, dtype>::identity();
        tree[origin].tf() = new_tf;
        tree.at(origin).tf() = new_tf;

        auto rootNode_1 = tree[origin];
        auto rootNode_2 = tree[origin];
        auto rootNode_3 = tree.at(origin);
        auto rootNode_4 = tree.at(origin);

        tree.clear();
        ASSERT_EQ(tree.root().key(), origin);
    }
};


TEST(t_tf_tree, api_test) {
    [[maybe_unused]]auto apiTest = rtl::test::RangeTypesTypes<APITest, RANGE_AND_DTYPES>::with<TYPES>{};
}


template<int N, typename dtype, typename K>
struct GeneralTFTest {
    static void testFunction() {


        auto keyGen = KeysGenerator<K>{keyN};
        auto keys = keyGen.generateKyes();
        auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);

        using General3DTf = rtl::GeneralTf<rtl::RigidTfND<3,double>, rtl::TranslationND<3, double>, rtl::RotationND<3, double>>;
        rtl::TfTree<std::string, General3DTf> generalTree{origin};

        auto rigid = rtl::RigidTfND<3, double>::random(generator);
        auto rot = rtl::RotationND<3, double>::random(generator);
        auto trans = rtl::TranslationND<3, double>::random(generator);

        /*
                           origin
                          /      \
                       trans     rot
                        /          \
                      1             2
                     /
                  rigidTf
                   /
                 3
        */

        generalTree.insert(key_1, trans, origin);
        generalTree.insert(key_2, rot, origin);
        generalTree.insert(key_3, rigid, key_1);

        auto chain_3_2 = generalTree.tf(key_3, key_2);

        auto tf_3_2 = rot(trans.inverted()(rigid.inverted()));
        ASSERT_EQ(CompareTfsEqual((rtl::RigidTfND<3, double>) chain_3_2.squash(), tf_3_2 ), true);
    }
};


TEST(t_tf_tree, generalTfTest) {
    [[maybe_unused]]auto generalTfTests = rtl::test::RangeTypesTypes<GeneralTFTest, RANGE_AND_DTYPES>::with<std::string>{};
}


TEST(t_tf_tree, str_cmp_vs_stc_hash) {

    auto keyGen = KeysGenerator<std::string>{2};
    auto keys = keyGen.generateKyes();
    auto a = origin;
    auto b = key_1;

    auto t1 = std::chrono::high_resolution_clock::now();
    for(size_t i = 0 ; i < 10000000 ; i++) {
        if( a == b ) {}
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    std::cout << "Str cmp duration: " << duration << " ms" << std::endl;

    auto hasher = std::hash<std::string>{};
    t1 = std::chrono::high_resolution_clock::now();
    for(size_t i = 0 ; i < 10000000 ; i++) {
        auto ha = hasher(a);
        auto hb = hasher(b);
        if( ha == hb ) {}
    }
    t2 = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    std::cout << "Str hash and int cmp duration: " << duration << " ms" << std::endl;;
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

