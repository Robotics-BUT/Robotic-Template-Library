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
#include <rtl/tf/GeneralTf.h>

#include "tf_test/tf_comparison.h"

#define EXPAND_TYPES_3D(x) rtl::Translation3D<x>, rtl::Rotation3D<x>, rtl::RigidTf3D<x>
#define EXPAND_TYPES_2D(x) rtl::Translation2D<x>, rtl::Rotation2D<x>, rtl::RigidTf2D<x>
#define EXPAND_TYPES(x) EXPAND_TYPES_3D(x), EXPAND_TYPES_2D(x)
#define ALL_TYPES EXPAND_TYPES(double), EXPAND_TYPES(float)

template <typename ...Tfs>
struct TestInit
{
    static void testFunction()
    {
        auto gtf = rtl::GeneralTf<Tfs...>();
    }
};

TEST(general_tf, init) {
    TestInit<ALL_TYPES>::testFunction();
}



template <typename ...Tfs>
struct TestConstructors
{
    static void testFunction()
    {
        auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);
        auto tf3d = rtl::RigidTfND<3, double>::random(generator);
        auto gtf = rtl::GeneralTf<Tfs...>( tf3d );

        ASSERT_EQ(CompareTfsEqual( gtf.template tf<rtl::RigidTfND<3, double>>(), tf3d), true);

        auto gtf_cp = rtl::GeneralTf<Tfs...>( gtf );
        ASSERT_EQ(CompareTfsEqual( gtf_cp.template tf<rtl::RigidTfND<3, double>>(), tf3d), true);

        rtl::GeneralTf<Tfs...> gtf_mv{gtf};
        ASSERT_EQ(CompareTfsEqual( gtf_mv.template tf<rtl::RigidTfND<3, double>>(), tf3d), true);
    }
};

TEST(general_tf, constructors) {
    TestConstructors<ALL_TYPES>::testFunction();
}



template <typename ...Tfs>
struct TestAssignOperator
{
    static void testFunction()
    {

        auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);
        auto tf3d = rtl::RigidTfND<3, double>::random(generator);
        auto gtf = rtl::GeneralTf<Tfs...>( tf3d );

        auto gtf_cp = gtf;
        ASSERT_EQ(CompareTfsEqual( gtf_cp.template tf<rtl::RigidTfND<3, double>>(), tf3d), true);

        auto gtf_mv = rtl::GeneralTf<Tfs...>( tf3d );
        ASSERT_EQ(CompareTfsEqual( gtf_mv.template tf<rtl::RigidTfND<3, double>>(), tf3d), true);

        auto tf2f = rtl::RigidTfND<2, float>::random(generator);
        gtf_mv = tf2f;
        ASSERT_EQ(CompareTfsEqual( gtf_mv.template tf<rtl::RigidTfND<2, float>>(), tf2f), true);
    }
};

TEST(general_tf, assign_operator) {

    TestAssignOperator<ALL_TYPES>::testFunction();
}


template <typename ...Tfs>
struct TestIndex
{
    static void testFunction()
    {
        testTranslations();
        testRotations();
        testTfs();
    }

    static void testTranslations() {
        auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);

        auto t3d = rtl::TranslationND<3, double>::random(generator);
        auto gtf = rtl::GeneralTf<Tfs...>( t3d );
        ASSERT_EQ(CompareTransEqual( gtf.template tf<rtl::TranslationND<3, double>>(), t3d), true);
        ASSERT_EQ(gtf.index(), 0);

        auto t2d = rtl::TranslationND<2, double>::random(generator);
        gtf = rtl::GeneralTf<Tfs...>( t2d );
        ASSERT_EQ(CompareTransEqual( gtf.template tf<rtl::TranslationND<2, double>>(), t2d), true);
        ASSERT_EQ(gtf.index(), 3);

        auto t3f = rtl::TranslationND<3, float>::random(generator);
        gtf = rtl::GeneralTf<Tfs...>( t3f );
        ASSERT_EQ(CompareTransEqual( gtf.template tf<rtl::TranslationND<3, float>>(), t3f), true);
        ASSERT_EQ(gtf.index(), 6);

        auto t2f = rtl::TranslationND<2, float>::random(generator);
        gtf = rtl::GeneralTf<Tfs...>( t2f );
        ASSERT_EQ(CompareTransEqual( gtf.template tf<rtl::TranslationND<2, float>>(), t2f), true);
        ASSERT_EQ(gtf.index(), 9);
    }

    static void testRotations() {
        auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);

        auto t3d = rtl::RotationND<3, double>::random(generator);
        auto gtf = rtl::GeneralTf<Tfs...>( t3d );
        ASSERT_EQ(CompareRotsEqual( gtf.template tf<rtl::RotationND<3, double>>(), t3d), true);
        ASSERT_EQ(gtf.index(), 1);

        auto t2d = rtl::RotationND<2, double>::random(generator);
        gtf = rtl::GeneralTf<Tfs...>( t2d );
        ASSERT_EQ(CompareRotsEqual( gtf.template tf<rtl::RotationND<2, double>>(), t2d), true);
        ASSERT_EQ(gtf.index(), 4);

        auto t3f = rtl::RotationND<3, float>::random(generator);
        gtf = rtl::GeneralTf<Tfs...>( t3f );
        ASSERT_EQ(CompareRotsEqual( gtf.template tf<rtl::RotationND<3, float>>(), t3f), true);
        ASSERT_EQ(gtf.index(), 7);

        auto t2f = rtl::RotationND<2, float>::random(generator);
        gtf = rtl::GeneralTf<Tfs...>( t2f );
        ASSERT_EQ(CompareRotsEqual( gtf.template tf<rtl::RotationND<2, float>>(), t2f), true);
        ASSERT_EQ(gtf.index(), 10);
    }

    static void testTfs() {
        auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);

        auto t3d = rtl::RigidTfND<3, double>::random(generator);
        auto gtf = rtl::GeneralTf<Tfs...>( t3d );
        ASSERT_EQ(CompareTfsEqual( gtf.template tf<rtl::RigidTfND<3, double>>(), t3d), true);
        ASSERT_EQ(gtf.index(), 2);

        auto t2d = rtl::RigidTfND<2, double>::random(generator);
        gtf = rtl::GeneralTf<Tfs...>( t2d );
        ASSERT_EQ(CompareTfsEqual( gtf.template tf<rtl::RigidTfND<2, double>>(), t2d), true);
        ASSERT_EQ(gtf.index(), 5);

        auto t3f = rtl::RigidTfND<3, float>::random(generator);
        gtf = rtl::GeneralTf<Tfs...>( t3f );
        ASSERT_EQ(CompareTfsEqual( gtf.template tf<rtl::RigidTfND<3, float>>(), t3f), true);
        ASSERT_EQ(gtf.index(), 8);

        auto t2f = rtl::RigidTfND<2, float>::random(generator);
        gtf = rtl::GeneralTf<Tfs...>( t2f );
        ASSERT_EQ(CompareTfsEqual( gtf.template tf<rtl::RigidTfND<2, float>>(), t2f), true);
        ASSERT_EQ(gtf.index(), 11);
    }
};

TEST(general_tf, index) {

    TestIndex<ALL_TYPES>::testFunction();
}

template <typename ...Tfs>
struct TestVisitor
{
    static void testFunction()
    {
        auto index_gen = rtl::test::Random::uniformCallable(0ul, sizeof...(Tfs) - 1);
        auto element_gen = rtl::test::Random::uniformCallable(-1.0, 1.0);
        static const int dims[] = {Tfs::dimensionality()...};
        auto get_dim = [](auto &&tr) { return tr.dimensionality(); };

        for (int i = 0; i < 50; i++)
        {
            auto gtf = rtl::GeneralTf<Tfs...>::random(element_gen, index_gen);
            ASSERT_EQ(gtf.visit(get_dim), dims[gtf.index()]);
        }
    }
};

TEST(general_tf, visitor) {

    TestVisitor<ALL_TYPES>::testFunction();
}

template <typename ...Tfs>
struct TestInversion
{
    static void testFunction()
    {
        auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);

        auto t3d = rtl::RigidTfND<3, double>::random(generator);
        auto gtf = rtl::GeneralTf<Tfs...>( t3d );

        ASSERT_EQ(CompareTfsEqual(gtf.template tf<rtl::RigidTfND<3, double>>(), t3d), true);
        ASSERT_EQ(CompareTfsEqual(gtf.inverted().template tf<rtl::RigidTfND<3, double>>(), t3d.inverted()), true);

        gtf.invert();
        ASSERT_EQ(CompareTfsEqual(gtf.template tf<rtl::RigidTfND<3, double>>(), t3d.inverted()), true);
        ASSERT_EQ(CompareTfsEqual(gtf.inverted().template tf<rtl::RigidTfND<3, double>>(), t3d), true);
    }
};

TEST(general_tf, inversion) {

    TestInversion<ALL_TYPES>::testFunction();
}



template <typename ...Tfs>
struct GetterTest
{
    static void testFunction()
    {
        auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);

        auto t3d = rtl::RigidTfND<3, double>::random(generator);
        auto gtf = rtl::GeneralTf<Tfs...>( t3d );
        ASSERT_EQ(CompareTfsEqual(gtf.template tf<rtl::RigidTfND<3, double>>(), t3d), true);

        for (size_t i = 0 ; i < 10 ; i++) {
            gtf = rtl::RigidTfND<3, double>::random(generator);
            CompareTfsNotEqual(gtf.template tf<rtl::RigidTfND<3, double>>(), t3d);

            t3d = rtl::RigidTfND<3, double>::random(generator);
            gtf = t3d;
            ASSERT_EQ(CompareTfsEqual(gtf.template tf<rtl::RigidTfND<3, double>>(), t3d), true);
        }
    }
};

TEST(general_tf, getter) {

    GetterTest<ALL_TYPES>::testFunction();
}


template <typename ...Tfs>
struct TestFunctor
{
    static void testFunction()
    {
        auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);

        std::vector<rtl::RigidTfND<3, double>> tfVector;
        std::vector<rtl::GeneralTf<Tfs...>> gtfVector;

        auto aggregated = rtl::RigidTfND<3, double>::identity();
        for (size_t i = 0 ; i < 10 ; i++) {
            tfVector.emplace_back(rtl::RigidTfND<3, double>::random(generator));
            gtfVector.emplace_back(rtl::GeneralTf<Tfs...>( tfVector.at(i) ));
            aggregated = tfVector.at(i)(aggregated);
        }


        {
            size_t i = 0;
            for (const auto &tf : tfVector) {
                ASSERT_EQ(CompareTfsEqual(tf, tfVector.at(i++)), true);
            }
        }

        auto aggregated2 = rtl::RigidTfND<3, double>::identity();
        for(const auto& tf : tfVector) {
            aggregated2 = tf(aggregated2);
        }
        ASSERT_EQ(CompareTfsEqual(aggregated, aggregated2), true);


        auto gtfAggregated = rtl::GeneralTf<Tfs...>( rtl::RigidTfND<3, double>::identity());
        for(const auto& gtf : gtfVector) {
            gtfAggregated = gtf.template operator()<rtl::RigidTfND<3, double>>(gtfAggregated);
        }
        ASSERT_EQ(CompareTfsEqual(aggregated, gtfAggregated.template tf<rtl::RigidTfND<3, double>>()), true);
    }
};

TEST(general_tf, functor) {

    TestFunctor<ALL_TYPES>::testFunction();
}


template <typename ...Tfs>
struct TestTransform
{
    static void testFunction()
    {
        auto generator = rtl::test::Random::uniformCallable<double>(-1.0, 1.0);

        std::vector<rtl::RigidTfND<3, double>> tfVector;
        std::vector<rtl::GeneralTf<Tfs...>> gtfVector;

        auto aggregated1 = rtl::RigidTfND<3, double>::identity();
        auto aggregated2 = rtl::RigidTfND<3, double>::identity();
        for (size_t i = 0 ; i < 10 ; i++) {
            tfVector.emplace_back(rtl::RigidTfND<3, double>::random(generator));
            gtfVector.emplace_back(rtl::GeneralTf<Tfs...>( tfVector.at(i) ));
            aggregated1.transform(tfVector.at(i));
            aggregated2 = aggregated2.transformed(tfVector.at(i));
        }
        ASSERT_EQ(CompareTfsEqual(aggregated1, aggregated2), true);

        auto gtfAggregated = rtl::GeneralTf<Tfs...>( rtl::RigidTfND<3, double>::identity());
        for(const auto& gtf : gtfVector) {
            gtfAggregated.transform(gtf);
        }
        ASSERT_EQ(CompareTfsEqual(aggregated1, gtfAggregated.template tf<rtl::RigidTfND<3, double>>()), true);
    }
};

TEST(general_tf, transform) {

    TestTransform<ALL_TYPES>::testFunction();
}



int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
