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

#include "tf_test/tf_comparison.h"
#include "tf_test/key_generator.h"


template<int N, typename dtype, typename T>
struct TestInit {
    static void testFunction() {

        rtl::TfChain<rtl::RigidTfND<N, dtype>> chain;
    }
};


TEST(t_tf_tree, init) {
    [[maybe_unused]]auto keyGenTest = rtl::test::RangeTypesTypes<TestInit, RANGE_AND_DTYPES>::with<TYPES>{};
}


template<int N, typename dtype, typename T>
struct TestConstructors {
    static void testFunction() {

        auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);

        std::list<rtl::RigidTfND<N, dtype>> tfList;
        tfList.push_back(rtl::RigidTfND<N, dtype>::random(generator));

        rtl::TfChain<rtl::RigidTfND<N, dtype>> chain;
        rtl::TfChain<rtl::RigidTfND<N, dtype>> chain_cp{chain};
        rtl::TfChain<rtl::RigidTfND<N, dtype>> chain_mv{ rtl::TfChain<rtl::RigidTfND<N, dtype>>{} };

        rtl::TfChain<rtl::RigidTfND<N, dtype>> chain_list {tfList};
        rtl::TfChain<rtl::RigidTfND<N, dtype>> chain_list_cp{chain_list};
        rtl::TfChain<rtl::RigidTfND<N, dtype>> chain_list_mv{ rtl::TfChain<rtl::RigidTfND<N, dtype>>{tfList} };

        ASSERT_EQ(CompareTfsEqual(chain.squash(), chain_cp.squash()), true);
        ASSERT_EQ(CompareTfsEqual(chain.squash(), chain_mv.squash()), true);

        ASSERT_EQ(CompareTfsEqual(chain_list.squash(), chain_list_cp.squash()), true);
        ASSERT_EQ(CompareTfsEqual(chain_list.squash(), chain_list_mv.squash()), true);

        ASSERT_EQ(CompareTfsNotEqual(chain.squash(), chain_list.squash()), true);
    }
};


TEST(t_tf_tree, constructors) {
    [[maybe_unused]]auto constructorTest = rtl::test::RangeTypesTypes<TestConstructors, RANGE_AND_DTYPES>::with<TYPES>{};
}


template<int N, typename dtype, typename T>
struct TestLists {
    static void testFunction() {

        auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);

        std::list<rtl::RigidTfND<N, dtype>> tfList;
        tfList.push_back(rtl::RigidTfND<N, dtype>::random(generator));

        auto identity = rtl::RigidTfND<N, dtype>::identity();

        auto tf1 = rtl::RigidTfND<N, dtype>::random(generator);
        auto tf2 = rtl::RigidTfND<N, dtype>::random(generator);
        auto tf3 = rtl::RigidTfND<N, dtype>::random(generator);

        auto list = std::list<rtl::RigidTfND<N, dtype>>{tf1, tf2, tf3};
        auto chain = rtl::TfChain<rtl::RigidTfND<N, dtype>>{list};

        ASSERT_EQ(CompareTfsEqual(chain.squash(), tf3(tf2(tf1))), true);
        ASSERT_EQ(CompareTfsEqual(chain(identity), tf3(tf2(tf1))), true);

        auto list2 = chain.list();

        auto aggregated1 = rtl::RigidTfND<N, dtype>::identity();
        auto aggregated2 = rtl::RigidTfND<N, dtype>::identity();
        for (const auto& tf : list) {aggregated1 = tf(aggregated1);}
        for (const auto& tf : list2) {aggregated2 = tf(aggregated2);}

        ASSERT_EQ(CompareTfsEqual(aggregated1, aggregated2), true);
    }
};


TEST(t_tf_tree, list_and_tfs) {
    [[maybe_unused]]auto lsits_test = rtl::test::RangeTypesTypes<TestLists, RANGE_AND_DTYPES>::with<TYPES>{};
}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

