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

#include "rtl/Transformation.h"
#include "rtl/io/StdLib.h"
#include "rtl/Test.h"

int main()
{
    using GTf = rtl::GeneralTf<rtl::Translation2f, rtl::Rotation2f, rtl::RigidTf2f>;
    using TRT = rtl::GeneralTf<rtl::Translation2f, rtl::Rotation2f, rtl::Translation3f>;
    using VR = typename rtl::VariantResultOTs<rtl::Vector2f, rtl::Translation2f, rtl::Rotation2f, rtl::RigidTf2f>;

    std::variant<rtl::Translation2f> vtf = rtl::Translation2f(1, 1);

    GTf tr1(rtl::Translation2f(1, 1));
    GTf tr_eq = tr1;
    std::cout<<tr_eq<<std::endl;

    GTf tr_mv = GTf(rtl::Translation2f(1, 1));
    std::cout<<tr_mv<<std::endl;

    auto t = rtl::Translation2f(1, 1);
    t.setTrVecX(10);

    GTf gtf;
    auto x = gtf.tf<rtl::Translation2f>();
    std::cout<<x<<std::endl;
    gtf = t;
    rtl::Translation2f tr2f = gtf;

    //rtl::Rotation2f rot2f = gtf; // Correctly throws an exception because of different alternative in the GTf.
    //rtl::Vector2f vec2f = gtf;   // Correctly fails on static assert, because rtl::Vector2f is not even an alternative in GTf.

    std::cout<<tr2f<<std::endl;
    std::cout<<gtf<<std::endl;
    std::cout<<t<<std::endl;
    std::cout<<gtf(rtl::Vector2f(1, 1))<<std::endl;
    std::cout<<gtf.tf<rtl::Translation2f>()(rtl::Translation2f(1, 1))<<std::endl;

    //rtl::Rotation2f ro_gtf = gtf(rtl::Translation2f(1, 1)); // Correctly fails on static_assert, because rtl::Rotation2f cannot appear as a result when any transformation from GTf is applied on rtl::Translation2f.
    //rtl::RigidTf2f ri_gtf = gtf(rtl::Translation2f(1, 1));    // Correctly throws, because gtf now contains rtl::Translation2f and after application on rtl::Translation2f the result is rtl::Translation2f again.

    rtl::Translation2f t_gtf = gtf(rtl::Translation2f(1, 1));
    std::cout<<t_gtf<<std::endl;
    std::cout<<gtf(gtf(gtf))<<std::endl;
    gtf.transform(rtl::Translation2f(10, 10));
    gtf.transform(gtf);
    std::cout<<gtf<<std::endl;
    std::cout<<gtf.transformed(gtf)<<std::endl;
    std::cout<<gtf.transformed(rtl::Translation2f(1, 10))<<std::endl;
    std::cout<<gtf.inverted()<<std::endl;
    gtf.invert();
    std::cout<<gtf<<std::endl;


    auto gtf_identity = GTf::identity();
    std::cout<<gtf_identity << " " << rtl::test::type<decltype(gtf_identity)>::description()<<std::endl;
    auto gtf_identity2 = rtl::GeneralTf<rtl::Vector2f, rtl::Translation2f>::identity();
    std::cout<<gtf_identity2 << " " << rtl::test::type<decltype(gtf_identity2)>::description()<<std::endl;
    //auto gtf_identity3 = rtl::GeneralTf<rtl::Vector2f, rtl::LineSegment2f>::identity(); // Correctly fails on static_assert since none of the given types provide a static identity() function.

    auto el_gen = rtl::test::Random::uniformCallable(-1.0f, 1.0f);
    auto gtf_rand = GTf::random(el_gen);
    std::cout<<gtf_rand << " " << rtl::test::type<decltype(gtf_rand)>::description()<<std::endl;

    auto tf_gen = rtl::test::Random::uniformCallable(0, 2);
    for (int i = 0; i < 20; i++)
    {
        auto trr_rand = GTf::random(el_gen, tf_gen);
        std::cout<< "Alternative: " << trr_rand.index()<<" Transformation" << trr_rand <<std::endl;
    }

    rtl::TfTree<std::string, rtl::Translation2f> tree("root");
    tree.insert("pos1", rtl::Translation2f(1, 1), "root");
    tree.insert("pos2", rtl::Translation2f(2, 1), "root");
    tree.insert("pos11", rtl::Translation2f(1, 2), "pos1");
    tree.insert("pos12", rtl::Translation2f(1, 3), "pos1");
    tree.insert("pos13", rtl::Translation2f(1, 4), "pos1");
    std::cout<<tree<<std::endl;

    std::cout<<tree["pos11"]<<std::endl;
    std::cout<<tree.at("pos2")<<std::endl;

    auto tf_from_to = tree.tf("pos12", "pos2");
    std::cout<<"TfChain:"<<std::endl;
    for (auto &l : tf_from_to.list())
        std::cout<<l<<std::endl;
    tf_from_to = tree.tf("pos2", "pos12");
    std::cout<<"TfChain:"<<std::endl;
    for (auto &l : tf_from_to.list())
        std::cout<<l<<std::endl;

    tree.erase("pos12");
    std::cout<<tree<<std::endl;

    tree.erase("pos1");
    std::cout<<tree<<std::endl;

    tree.erase("pos2");
    std::cout<<tree<<std::endl;

    std::cout<< tree.root().key() << " " << tree.root().parent()->key() << std::endl;

    TRT tr(rtl::Translation2f::identity());

    rtl::Vector2f v2f(1, 1);
    v2f = tr(v2f);
    std::cout<<v2f<<std::endl;

    VR vr(rtl::Vector2f::ones());

    rtl::TfChain<GTf> chain({GTf(rtl::Translation2f(1, 2)), GTf(rtl::Rotation2f(rtl::C_PIf)), GTf(rtl::Translation2f(3, 1))});
    std::cout<<v2f<<std::endl;
    std::cout<<chain(v2f)<<std::endl;

    tr2f = rtl::Translation2f(1, 1);
    std::cout<<tr2f<<std::endl;

    //std::cout<<(rtl::Vector2f)chain(tr2f)<<std::endl; // Correctly fails on static_assert, because rtl::Vector2f cannot appear as a result when any transformation from GTf is applied on rtl::Translation2f.
    //std::cout<<(rtl::Translation2f)chain(tr2f)<<std::endl; // Correctly throws, because the result of application of the chain on rtl::Translation2f is rtl::Rigid2f (rtl::Translation2f is within possible result types, but bad alternative in this case).
    std::cout<<(rtl::RigidTf2f)chain(tr2f)<<std::endl;

    auto chain_squashed = chain.squash();
    std::cout<<chain_squashed<<std::endl;

    return 0;
}

