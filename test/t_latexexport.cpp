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

#include "rtl/io/LaTeXDoc.h"

int main()
{
    rtl::LaTeXDoc ld("t_latexexport_out", "base_test");
    ld.setRemoveTmpDir([](const std::string &){ return true; });

    rtl::LaTeXTikz3D le3;
    le3.setExportSize(4, 4);
    rtl::RigidTf3f view_tr = rtl::RigidTf3f::identity();
    auto rot_view_lambda_const_dist = [&le3](int i_max, int i) -> rtl::LaTeXTikz3D& { le3.setView(75, rtl::RigidTf3f(2.0f * rtl::C_PIf / i_max * i, rtl::Vector3f(1, 1, 1), rtl::Vector3f(0, 0, -2))); return le3; };
    auto rot_view_lambda_frame_fit = [&le3](int i_max, int i) -> rtl::LaTeXTikz3D& { le3.setView(45, rtl::RigidTf3f(2.0f * rtl::C_PIf / i_max * i, rtl::Vector3f(1, 1, 1), rtl::Vector3f::nan())); return le3; };

    int i_max = 100;
    float a = rtl::C_PIf / 20.0f, b = 0.008f;
    for (int i = 0; i < i_max; i++)
        le3.addMark(rtl::Vector3f(0.5 * std::cos(a * (float)i), 0.5 * std::sin(a * (float)i), b * (float)i), "style={draw=black,fill=green}", rtl::LaTeXTikz3D::latex_mark_dot, 0.0f, 0.05f);
    le3.addLine(rtl::LineSegment3f(rtl::Vector3f(0, 0, -0.5), rtl::Vector3f(0, 0, 1.5)), "style={draw=black,thick}");
    ld.addGridLE(rot_view_lambda_const_dist, 4, 20);

    le3.clearAll();
    le3.setExportSize(10, 10);
    le3.setMinPlotRegion(rtl::Vector3f::zeros(), rtl::Vector3f(1, 1, 1));
    view_tr.setTrVecZ(-3.0);
    le3.setView(75, view_tr);
    le3.addAxis("style={draw=red,thick,->}", "%d", rtl::LaTeXTikz3D::position_left, 1, rtl::Vector3f::zeros(), rtl::Vector3f::baseX());
    le3.addAxis("style={draw=green,thick,->}", "%d", rtl::LaTeXTikz3D::position_left, 1, rtl::Vector3f::zeros(), rtl::Vector3f::baseY());
    le3.addAxis("style={draw=blue,thick,->}", "%d", rtl::LaTeXTikz3D::position_left, 1, rtl::Vector3f::zeros(), rtl::Vector3f::baseZ());
    ld.addLE(le3, "Axes base position");
    ld.addGridLE(rot_view_lambda_const_dist, 4, 20);

    le3.clearAll();
    le3.setExportSize(4, 4);
    std::vector<rtl::LineSegment3f> cube_wire = {rtl::LineSegment3f(rtl::Vector3f(-0.5f, -0.5f, -0.5f), rtl::Vector3f(-0.5f, 0.5f, -0.5f)),
                                                 rtl::LineSegment3f(rtl::Vector3f(-0.5f, 0.5f, -0.5f), rtl::Vector3f(0.5f, 0.5f, -0.5f)),
                                                 rtl::LineSegment3f(rtl::Vector3f(0.5f, 0.5f, -0.5f), rtl::Vector3f(0.5f, -0.5f, -0.5f)),
                                                 rtl::LineSegment3f(rtl::Vector3f(0.5f, -0.5f, -0.5f), rtl::Vector3f(-0.5f, -0.5f, -0.5f)),

                                                 rtl::LineSegment3f(rtl::Vector3f(-0.5f, -0.5f, -0.5f), rtl::Vector3f(-0.5f, -0.5f, 0.5f)),
                                                 rtl::LineSegment3f(rtl::Vector3f(-0.5f, 0.5f, -0.5f), rtl::Vector3f(-0.5f, 0.5f, 0.5f)),
                                                 rtl::LineSegment3f(rtl::Vector3f(0.5f, 0.5f, -0.5f), rtl::Vector3f(0.5f, 0.5f, 0.5f)),
                                                 rtl::LineSegment3f(rtl::Vector3f(0.5f, -0.5f, -0.5f), rtl::Vector3f(0.5f, -0.5f, 0.5f)),

                                                 rtl::LineSegment3f(rtl::Vector3f(-0.5f, 0.5f, 0.5f), rtl::Vector3f(0.5f, 0.5f, 0.5f)),
                                                 rtl::LineSegment3f(rtl::Vector3f(0.5f, 0.5f, 0.5f), rtl::Vector3f(0.5f, -0.5f, 0.5f)),
                                                 rtl::LineSegment3f(rtl::Vector3f(-0.5f, -0.5f, 0.5f), rtl::Vector3f(-0.5f, 0.5f, 0.5f)),
                                                 rtl::LineSegment3f(rtl::Vector3f(0.5f, -0.5f, 0.5f), rtl::Vector3f(-0.5f, -0.5f, 0.5f))};
    le3.addLines(cube_wire, "style={draw=black,thin}");
    ld.addGridLE(rot_view_lambda_const_dist, 4, 20);

    le3.setFrameStyle("draw=black, thick");
    ld.addGridLE(rot_view_lambda_frame_fit, 4, 20);

    le3.clearAll();
    rtl::Polygon3Df square(rtl::Vector3f::baseX(), 0);
    square.addPoint(rtl::Vector3f(0, 0.5f, 0.5f));
    square.addPoint(rtl::Vector3f(0, 0.5f, -0.5f));
    square.addPoint(rtl::Vector3f(0, -0.5f, -0.5f));
    square.addPoint(rtl::Vector3f(0, -0.5f, 0.5f));
    le3.addFace(rtl::RigidTf3f(rtl::C_PIf, rtl::Vector3f::baseY(), rtl::Vector3f(-0.5, 0, 0))(square), "style={fill=red}", "style={fill=black}", "");
    le3.addFace(rtl::RigidTf3f(-rtl::C_PIf / 2.0f, rtl::Vector3f::baseZ(), rtl::Vector3f(0, -0.5, 0))(square), "style={fill=green}", "style={fill=black}", "");
    le3.addFace(rtl::RigidTf3f(rtl::C_PIf / 2.0f, rtl::Vector3f::baseY(), rtl::Vector3f(0, 0, -0.5))(square), "style={fill=blue}", "style={fill=black}", "");
    le3.addFace(rtl::RigidTf3f(0, rtl::Vector3f::baseY(), rtl::Vector3f(0.5, 0, 0))(square), "style={fill=yellow}", "style={fill=black}", "");
    le3.addFace(rtl::RigidTf3f(rtl::C_PIf / 2.0f, rtl::Vector3f::baseZ(), rtl::Vector3f(0, 0.5, 0))(square), "style={fill=cyan}", "style={fill=black}", "");
    le3.addFace(rtl::RigidTf3f(-rtl::C_PIf / 2.0f, rtl::Vector3f::baseY(), rtl::Vector3f(0, 0, 0.5))(square), "style={fill=magenta}", "style={fill=black}", "");
    ld.addGridLE(rot_view_lambda_const_dist, 4, 20);

    le3.clearAll();
    le3.addFace(square, "style={fill=red}", "style={fill=red}", "");
    le3.addFace(rtl::RigidTf3f(rtl::C_PIf / 4.0f, rtl::Vector3f::baseY(), rtl::Vector3f::zeros())(square), "style={fill=blue}", "style={fill=blue}", "");
    ld.addGridLE(rot_view_lambda_frame_fit, 4, 20);

    rtl::Polygon3Df cutting_square(rtl::Vector3f::baseZ(), 0);
    cutting_square.addPoint(rtl::Vector3f(5, 5, 0));
    cutting_square.addPoint(rtl::Vector3f(5, -5, 0));
    cutting_square.addPoint(rtl::Vector3f(-5, -5, 0));
    cutting_square.addPoint(rtl::Vector3f(-5, 5, 0));
    cutting_square.transform(rtl::Rotation3f(rtl::C_PIf / 4, rtl::Vector3f::baseX()));

    le3.clearAll();
    le3.addFace(cutting_square, "style={fill=blue}", "style={fill=blue}", "");
    rtl::Polygon3Df obj0(rtl::Vector3f::baseZ(), 0);
    obj0.addPoint(rtl::Vector3f(4, 0, 0));
    obj0.addPoint(rtl::Vector3f(4, -4, 0));
    obj0.addPoint(rtl::Vector3f(-4, -4, 0));
    obj0.addPoint(rtl::Vector3f(-4, 4, 0));
    obj0.addPoint(rtl::Vector3f(0, 4, 0));
    obj0.addPoint(rtl::Vector3f(0, 0, 0));
    le3.addFace(obj0, "style={fill=red}", "style={fill=red}", "");
    ld.addGridLE(rot_view_lambda_frame_fit, 4, 20);

    le3.clearAll();
    le3.addFace(cutting_square, "style={fill=blue}", "style={fill=blue}", "");
    rtl::Polygon3Df obj1(rtl::Vector3f::baseZ(), 0);
    obj1.addPoint(rtl::Vector3f(-4, 4, 0));
    obj1.addPoint(rtl::Vector3f(-1, 4, 0));
    obj1.addPoint(rtl::Vector3f(-1, -1, 0));
    obj1.addPoint(rtl::Vector3f(1, -1, 0));
    obj1.addPoint(rtl::Vector3f(1, 4, 0));
    obj1.addPoint(rtl::Vector3f(4, 4, 0));
    obj1.addPoint(rtl::Vector3f(4, -4, 0));
    obj1.addPoint(rtl::Vector3f(-4, -4, 0));
    le3.addFace(obj1, "style={fill=red}", "style={fill=red}", "");
    ld.addGridLE(rot_view_lambda_frame_fit, 4, 20);

    le3.clearAll();
    le3.addFace(cutting_square, "style={fill=blue}", "style={fill=blue}", "");
    rtl::Polygon3Df obj2(rtl::Vector3f::baseZ(), 0);
    obj2.addPoint(rtl::Vector3f(-4, 4, 0));
    obj2.addPoint(rtl::Vector3f(-4, -4, 0));
    obj2.addPoint(rtl::Vector3f(4, -4, 0));
    obj2.addPoint(rtl::Vector3f(4, 4, 0));
    obj2.addPoint(rtl::Vector3f(-2, 4, 0));
    obj2.addPoint(rtl::Vector3f(-2, -2, 0));
    obj2.addPoint(rtl::Vector3f(2, -2, 0));
    obj2.addPoint(rtl::Vector3f(2, 2, 0));
    obj2.addPoint(rtl::Vector3f(0, 2, 0));
    obj2.addPoint(rtl::Vector3f(0, 1, 0));
    obj2.addPoint(rtl::Vector3f(1, 1, 0));
    obj2.addPoint(rtl::Vector3f(1, -1, 0));
    obj2.addPoint(rtl::Vector3f(-1, -1, 0));
    obj2.addPoint(rtl::Vector3f(-1, 3, 0));
    obj2.addPoint(rtl::Vector3f(3, 3, 0));
    obj2.addPoint(rtl::Vector3f(3, -3, 0));
    obj2.addPoint(rtl::Vector3f(-3, -3, 0));
    obj2.addPoint(rtl::Vector3f(-3, 4, 0));
    le3.addFace(obj2, "style={fill=red}", "style={fill=red}", "");
    ld.addGridLE(rot_view_lambda_frame_fit, 4, 20);


    le3.clearAll();
    le3.addFace(cutting_square, "style={fill=blue}", "style={fill=blue}", "");
    rtl::Polygon3Df obj3(rtl::Vector3f::baseZ(), 0);
    obj3.addPoint(rtl::Vector3f(-3, 0, 0));
    obj3.addPoint(rtl::Vector3f(-4, 2, 0));
    obj3.addPoint(rtl::Vector3f(-4, 4, 0));
    obj3.addPoint(rtl::Vector3f(1, 4, 0));
    obj3.addPoint(rtl::Vector3f(1, -2, 0));
    obj3.addPoint(rtl::Vector3f(2, -2, 0));
    obj3.addPoint(rtl::Vector3f(3, 0, 0));
    obj3.addPoint(rtl::Vector3f(4, -2, 0));
    obj3.addPoint(rtl::Vector3f(4, -4, 0));
    obj3.addPoint(rtl::Vector3f(-1, -4, 0));
    obj3.addPoint(rtl::Vector3f(-1, 2, 0));
    obj3.addPoint(rtl::Vector3f(-2, 2, 0));
    le3.addFace(obj3, "style={fill=red}", "style={fill=red}", "");
    ld.addGridLE(rot_view_lambda_frame_fit, 4, 20);


    le3.clearAll();
    le3.addFace(cutting_square, "style={fill=blue}", "style={fill=blue}", "");
    rtl::Polygon3Df obj4(rtl::Vector3f::baseZ(), 0);
    obj4.addPoint(rtl::Vector3f(-3, 0, 0));
    obj4.addPoint(rtl::Vector3f(-4, 2, 0));
    obj4.addPoint(rtl::Vector3f(-4, 4, 0));
    obj4.addPoint(rtl::Vector3f(4, 4, 0));
    obj4.addPoint(rtl::Vector3f(4, -4, 0));
    obj4.addPoint(rtl::Vector3f(-4, -4, 0));
    obj4.addPoint(rtl::Vector3f(-4, 0, 0));
    obj4.addPoint(rtl::Vector3f(-1, 0, 0));
    obj4.addPoint(rtl::Vector3f(-1, -1, 0));
    obj4.addPoint(rtl::Vector3f(1, -1, 0));
    obj4.addPoint(rtl::Vector3f(1, 2, 0));
    obj4.addPoint(rtl::Vector3f(-2, 2, 0));
    le3.addFace(obj4, "style={fill=red}", "style={fill=red}", "");
    ld.addGridLE(rot_view_lambda_frame_fit, 4, 20);


    le3.clearAll();
    le3.addFace(cutting_square, "style={fill=blue}", "style={fill=blue}", "");
    rtl::Polygon3Df obj5(rtl::Vector3f::baseZ(), 0);
    obj5.addPoint(rtl::Vector3f(-4, 4, 0));
    obj5.addPoint(rtl::Vector3f(-4, -4, 0));
    obj5.addPoint(rtl::Vector3f(4, -4, 0));
    obj5.addPoint(rtl::Vector3f(4, 4, 0));
    obj5.addPoint(rtl::Vector3f(3, 4, 0));
    obj5.addPoint(rtl::Vector3f(3, 0, 0));
    obj5.addPoint(rtl::Vector3f(1, 0, 0));
    obj5.addPoint(rtl::Vector3f(1, -2, 0));
    obj5.addPoint(rtl::Vector3f(-1, -2, 0));
    obj5.addPoint(rtl::Vector3f(-1, 0, 0));
    obj5.addPoint(rtl::Vector3f(-3, 0, 0));
    obj5.addPoint(rtl::Vector3f(-3, 4, 0));
    le3.addFace(obj5, "style={fill=red}", "style={fill=red}", "");
    ld.addGridLE(rot_view_lambda_frame_fit, 4, 20);


    auto colliding_cubes = [&le3, &square](int i_max, int i) -> rtl::LaTeXTikz3D&
    {
        float factor = float(i) / float(i_max);
        le3.setView(75, rtl::RigidTf3f(0.0f, rtl::Vector3f(1, 1, 1), rtl::Vector3f(0, 0, -3)));
        le3.clearAll();

        rtl::RigidTf3f c0_tr(- 2.0f * rtl::C_PIf * factor, rtl::Vector3f(1, 1, 1), rtl::Vector3f(0.6f - 1.2f * factor, 0, 0));
        le3.addFace(c0_tr(rtl::RigidTf3f(rtl::C_PIf, rtl::Vector3f::baseY(), rtl::Vector3f(-0.5, 0, 0))(square)), "style={fill=red}", "style={fill=black}", "");
        le3.addFace(c0_tr(rtl::RigidTf3f(-rtl::C_PIf / 2.0f, rtl::Vector3f::baseZ(), rtl::Vector3f(0, -0.5, 0))(square)), "style={fill=green}", "style={fill=black}", "");
        le3.addFace(c0_tr(rtl::RigidTf3f(rtl::C_PIf / 2.0f, rtl::Vector3f::baseY(), rtl::Vector3f(0, 0, -0.5))(square)), "style={fill=blue}", "style={fill=black}", "");
        le3.addFace(c0_tr(rtl::RigidTf3f(0, rtl::Vector3f::baseY(), rtl::Vector3f(0.5, 0, 0))(square)), "style={fill=yellow}", "style={fill=black}", "");
        le3.addFace(c0_tr(rtl::RigidTf3f(rtl::C_PIf / 2.0f, rtl::Vector3f::baseZ(), rtl::Vector3f(0, 0.5, 0))(square)), "style={fill=cyan}", "style={fill=black}", "");
        le3.addFace(c0_tr(rtl::RigidTf3f(-rtl::C_PIf / 2.0f, rtl::Vector3f::baseY(), rtl::Vector3f(0, 0, 0.5))(square)), "style={fill=magenta}", "style={fill=black}", "");

        rtl::RigidTf3f c1_tr(2.0f * rtl::C_PIf * factor, rtl::Vector3f(1, 1, 1), rtl::Vector3f(-0.6f + 1.2f * factor, 0, 0));
        le3.addFace(c1_tr(rtl::RigidTf3f(rtl::C_PIf, rtl::Vector3f::baseY(), rtl::Vector3f(-0.5, 0, 0))(square)), "style={fill=red}", "style={fill=black}", "");
        le3.addFace(c1_tr(rtl::RigidTf3f(-rtl::C_PIf / 2.0f, rtl::Vector3f::baseZ(), rtl::Vector3f(0, -0.5, 0))(square)), "style={fill=green}", "style={fill=black}", "");
        le3.addFace(c1_tr(rtl::RigidTf3f(rtl::C_PIf / 2.0f, rtl::Vector3f::baseY(), rtl::Vector3f(0, 0, -0.5))(square)), "style={fill=blue}", "style={fill=black}", "");
        le3.addFace(c1_tr(rtl::RigidTf3f(0, rtl::Vector3f::baseY(), rtl::Vector3f(0.5, 0, 0))(square)), "style={fill=yellow}", "style={fill=black}", "");
        le3.addFace(c1_tr(rtl::RigidTf3f(rtl::C_PIf / 2.0f, rtl::Vector3f::baseZ(), rtl::Vector3f(0, 0.5, 0))(square)), "style={fill=cyan}", "style={fill=black}", "");
        le3.addFace(c1_tr(rtl::RigidTf3f(-rtl::C_PIf / 2.0f, rtl::Vector3f::baseY(), rtl::Vector3f(0, 0, 0.5))(square)), "style={fill=magenta}", "style={fill=black}", "");

        return le3;
    };
    ld.addGridLE(colliding_cubes, 4, 20);


    le3.clearAll();
    std::vector<rtl::Vector3f> cube_vertices = {rtl::Vector3f(-0.5f, -0.5f, -0.5f), rtl::Vector3f(-0.5f, 0.5f, -0.5f), rtl::Vector3f(0.5f, 0.5f, -0.5f), rtl::Vector3f(0.5f, -0.5f, -0.5f),
                                                rtl::Vector3f(-0.5f, 0.5f, 0.5f), rtl::Vector3f(0.5f, 0.5f, 0.5f), rtl::Vector3f(-0.5f, -0.5f, 0.5f), rtl::Vector3f(0.5f, -0.5f, 0.5f)};
    le3.addMarks(cube_vertices, "style={draw=black,fill=white}", rtl::LaTeXTikz3D::latex_mark_dot, 0.0f, 0.05f);
    le3.addLines(cube_wire, "style={draw=black,thin}");
    le3.addFace(rtl::RigidTf3f(rtl::C_PIf, rtl::Vector3f::baseY(), rtl::Vector3f(-0.5, 0, 0))(square), "style={fill=red}", "style={fill=black}", "");
    le3.addFace(rtl::RigidTf3f(-rtl::C_PIf / 2.0f, rtl::Vector3f::baseZ(), rtl::Vector3f(0, -0.5, 0))(square), "style={fill=green}", "style={fill=black}", "");
    le3.addFace(rtl::RigidTf3f(rtl::C_PIf / 2.0f, rtl::Vector3f::baseY(), rtl::Vector3f(0, 0, -0.5))(square), "style={fill=blue}", "style={fill=black}", "");
    le3.addFace(rtl::RigidTf3f(0, rtl::Vector3f::baseY(), rtl::Vector3f(0.5, 0, 0))(square), "style={fill=yellow}", "style={fill=black}", "");
    le3.addFace(rtl::RigidTf3f(rtl::C_PIf / 2.0f, rtl::Vector3f::baseZ(), rtl::Vector3f(0, 0.5, 0))(square), "style={fill=cyan}", "style={fill=black}", "");
    le3.addFace(rtl::RigidTf3f(-rtl::C_PIf / 2.0f, rtl::Vector3f::baseY(), rtl::Vector3f(0, 0, 0.5))(square), "style={fill=magenta}", "style={fill=black}", "");
    ld.addGridLE(rot_view_lambda_const_dist, 4, 20);

    return 0;
}