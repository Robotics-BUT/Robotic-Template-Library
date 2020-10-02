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

#include <vector>

#include "rtl/Core.h"
#include "rtl/io/LaTeXDoc.h"

int main()
{
    // Create a LaTeX document for our plots in e_latexexport_out directory named example_outputs.pdf.
    rtl::LaTeXDoc ld("e_latexexport_out", "example_outputs");

    // rtl::LaTeXDoc runs commands for output compilation and file management. These can be altered by passing callable objects with desired behaviour and taking
    // required arguments (file names, paths, ...). This is e.g. a default callable for erasure of the temporary objects:
    ld.setRemoveTmpDir([](const std::string &dir) { return std::system(("rm -r " + dir).c_str()) >= 0; });

    // If we want to see the temporaries, we can alter the callable and use e.g. an empty lambda:
    ld.setRemoveTmpDir([](const std::string &) { return true; });

    // For drawing in 2D, we create an instance of a 2D exporter.
    rtl::LaTeXTikz2D le_lin;
    // Now we can set size of the output Tikz image.
    le_lin.setSize(7, 5);
    // Setup background grid style and period.
    le_lin.addGridH("style={draw=gray,dotted,thin}", rtl::C_PIf / 4.0f);
    le_lin.addGridV("style={draw=gray,dotted,thin}", 0.5f);
    // Setup axes style, number format, positioning of the numbers and numbering period.
    le_lin.addAxisX("style={draw=black,thick}", "%0.2f", rtl::LaTeXTikz2D::position_below | rtl::LaTeXTikz2D::position_right, rtl::C_PIf / 2.0f);
    le_lin.addAxisY("style={draw=black,thick}", "%0.1f", rtl::LaTeXTikz2D::position_above | rtl::LaTeXTikz2D::position_left, 0.5f);

    // Now we crate some data.
    std::vector<float> x_lin, sine, cos2, sin_plus_cos;
    for (float x = 0.0f; x < 7.0f; x += 0.2f)
        x_lin.push_back(x);
    for (auto x : x_lin)
    {
        sine.push_back(std::sin(x));
        cos2.push_back(std::cos(x) * std::cos(x));
        sin_plus_cos.push_back(std::sin(x) + std::cos(x));
    }

    // Plotting simple data is intuitive:
    le_lin.addPlot(x_lin, sine, "style={draw=green!40!gray, very thick}", "style={draw=green!40!gray, fill=white, very thick}", rtl::LaTeXTikz2D::latex_mark_dot);
    le_lin.addPlot(x_lin, cos2, "style={draw=yellow!40!gray, very thick}", "style={draw=yellow!40!gray, fill=white, very thick}", rtl::LaTeXTikz2D::latex_mark_mark);
    le_lin.addPlot(x_lin, sin_plus_cos, "style={draw=red!40!gray, very thick}", "style={draw=red!40!gray, fill=red!40!gray, very thick}", rtl::LaTeXTikz2D::latex_mark_dot, 0.5f);

    // And let us put the image into the output document. We could save the resulting .tex file separately with le_lin.writeTex(), but it would not compile it for us - rtl::LaTeXDoc will try to.
    ld.addLE(le_lin, "Simple trigonometric graphs.");


    // The second example uses logarithmic coordinates.
    rtl::LaTeXTikz2D le_log(rtl::LaTeXTikz2D::axis_type_log10, rtl::LaTeXTikz2D::axis_type_log10);
    // Setting up the exporter similar to the previous example:
    le_log.setSize(7, 5);
    le_log.setScaleY(0.3f);
    le_log.addGridH("style={draw=gray,dotted,thin}", 10);
    le_log.addGridV("style={draw=gray,dotted,thin}", 100);
    // Do not forget to move intersection of axes from [0, 0] - this would make mess in logarithmic scale.
    le_log.addAxisX("style={draw=black,thick}", "%0.0e", rtl::LaTeXTikz2D::position_below | rtl::LaTeXTikz2D::position_right, 10, 0.1f);
    le_log.addAxisY("style={draw=black,thick}", "%0.0e", rtl::LaTeXTikz2D::position_above | rtl::LaTeXTikz2D::position_left, 100, 1.0f);
    le_log.addDescriptionX("Processed items");
    le_log.addDescriptionY("Operations needed");

    // Another data generation section:
    std::vector<float> x_log = {2, 5, 10, 20, 50, 100, 200, 500, 1000};
    std::vector<float> logn, n, nlogn, nsq, expn;
    for (auto x : x_log)
    {
        logn.push_back(std::log(x));
        n.push_back(x);
        nlogn.push_back(x * std::log(x));
        nsq.push_back(x * x);
        expn.push_back(std::exp(x));
    }

    le_log.addPlot(x_log, logn, "style={draw=green!40!gray, very thick}");
    le_log.addPlot(x_log, n, "style={draw=yellow!40!gray, very thick}");
    le_log.addPlot(x_log, nlogn, "style={draw=orange!40!gray, very thick}");
    le_log.addPlot(x_log, nsq, "style={draw=red!40!gray, very thick}");
    // Let us truncate the exponential series to fit the plot.
    x_log.resize(3);
    x_log.push_back(std::log(nsq.back()));
    expn.resize(3);
    expn.push_back(std::exp(x_log.back()));
    le_log.addPlot(x_log, expn, "style={draw=purple!40!gray, very thick}");
    // Finally add the image to the document with appropriate description.
    ld.addLE(le_log, "Big-O computational complexity plot in logarithmic scale on axes.");


    // 2D drawing is more flexible than just charts:
    rtl::LaTeXTikz2D le_robot;
    le_robot.setSize(7, 7);
    le_robot.addGridH("style={draw=gray,dotted,thin}", 50);
    le_robot.addGridV("style={draw=gray,dotted,thin}", 50);

    // Generate edges of the room:
    std::vector<rtl::LineSegment2f> edges = {rtl::LineSegment2f(-200, -200, 200, -200), rtl::LineSegment2f(200, -200, 200, 200),
                                             rtl::LineSegment2f(200, 200, -200, 200), rtl::LineSegment2f(-200, 200, -200, -200)};
    le_robot.addEdges(edges, "style={draw=gray,line cap=round,line width=3pt}");

    // Add some obstacles:
    std::string obstacle_style = "style={draw={rgb,255:red,122;green,193;blue,67},fill={rgb,255:red,212;green,243;blue,157}, ultra thick}";
    le_robot.addCircle(rtl::Vector2f(-30, 50), 40, obstacle_style);
    le_robot.addRectangle(rtl::Vector2f(20, 10), rtl::Vector2f(50, 60), obstacle_style);
    le_robot.addTriangle(rtl::Vector2f(-70, -10), rtl::Vector2f(10, -15), rtl::Vector2f(-20, -55), obstacle_style);
    le_robot.addEllipse(rtl::Vector2f(50, -30), 15, 40, rtl::C_PIf / 4.0f, obstacle_style);

    // And a robot riding on a circular path around them:
    std::vector<rtl::Vector2f> path;
    for (int steps = 15, i = 0; i < steps; i++)
    {
        float angle = 2 * rtl::C_PIf * (float) i / (float) (steps);
        path.emplace_back(std::cos(angle) * 150, std::sin(angle) * 150);
    }
    le_robot.addPlot(path, "style={draw=gray, thick, dotted}", "style={draw=black, fill=white, very thick}", rtl::LaTeXTikz2D::latex_mark_robot);

    // The resulting map of the environment with robot's path is added to our results as well.
    ld.addLE(le_robot, "Robot's trajectory in a simulated environment.");


    // There is a LaTeX Tikz export for 3D as well:
    rtl::LaTeXTikz3D le3;

    // We will render a rotating cube. This is one face:
    rtl::Polygon3Df square(rtl::Vector3f::baseX(), 0);
    square.addPoint(rtl::Vector3f(0, 0.5f, 0.5f));
    square.addPoint(rtl::Vector3f(0, 0.5f, -0.5f));
    square.addPoint(rtl::Vector3f(0, -0.5f, -0.5f));
    square.addPoint(rtl::Vector3f(0, -0.5f, 0.5f));

    // These are vertices of the cube, we will mark them out.
    std::vector<rtl::Vector3f> cube_vertices = {rtl::Vector3f(-0.5f, -0.5f, -0.5f), rtl::Vector3f(-0.5f, 0.5f, -0.5f), rtl::Vector3f(0.5f, 0.5f, -0.5f), rtl::Vector3f(0.5f, -0.5f, -0.5f),
                                                rtl::Vector3f(-0.5f, 0.5f, 0.5f), rtl::Vector3f(0.5f, 0.5f, 0.5f), rtl::Vector3f(-0.5f, -0.5f, 0.5f), rtl::Vector3f(0.5f, -0.5f, 0.5f)};

    // Adding 3D point-like marks is similar to 2D plotting:
    le3.addMarks(cube_vertices, "style={draw=black,fill=white}", rtl::LaTeXTikz3D::latex_mark_dot, 0.0f, 0.05f);

    // Now we will add six faces of the cube. All of them are derived from the square above using rigid transformation. We could write the coordinates explicitly, but this is shorter.
    // Faces have styles for both sides and for edges, we use them all.
    le3.addFace(rtl::RigidTf3f(rtl::C_PIf, rtl::Vector3f::baseY(), rtl::Vector3f(-0.5, 0, 0))(square), "style={fill=red!30!gray}", "style={fill=black}", "style={draw=black,thin}");
    le3.addFace(rtl::RigidTf3f(-rtl::C_PIf / 2.0f, rtl::Vector3f::baseZ(), rtl::Vector3f(0, -0.5, 0))(square), "style={fill=green!30!gray}", "style={fill=black}", "style={draw=black,thin}");
    le3.addFace(rtl::RigidTf3f(rtl::C_PIf / 2.0f, rtl::Vector3f::baseY(), rtl::Vector3f(0, 0, -0.5))(square), "style={fill=blue!30!gray}", "style={fill=black}", "style={draw=black,thin}");
    le3.addFace(rtl::RigidTf3f(0, rtl::Vector3f::baseY(), rtl::Vector3f(0.5, 0, 0))(square), "style={fill=yellow!30!gray}", "style={fill=black}", "style={draw=black,thin}");
    le3.addFace(rtl::RigidTf3f(rtl::C_PIf / 2.0f, rtl::Vector3f::baseZ(), rtl::Vector3f(0, 0.5, 0))(square), "style={fill=cyan!30!gray}", "style={fill=black}", "style={draw=black,thin}");
    le3.addFace(rtl::RigidTf3f(-rtl::C_PIf / 2.0f, rtl::Vector3f::baseY(), rtl::Vector3f(0, 0, 0.5))(square), "style={fill=magenta!30!gray}", "style={fill=black}", "style={draw=black,thin}");

    // Now we could simply specify a view on the scene and render a single image, but LaTeXDoc allows to group many images into a grid. The lambda below takes reference to the LaTeX exporter
    // with out cube and changes its view pose with respect to parameters i_max and i. These correspond to total number of images in the grid and the number of actually requested image.
    // We could even use more exporters, but let us just plot twenty images of a wildly rotating cube:
    auto rot_view_lambda_const_dist = [&le3](int i_max, int i) -> rtl::LaTeXTikz3D&
            {
                auto rot0 = rtl::Rotation3f(rtl::C_PIf / 4.0f, rtl::Vector3f::baseX());
                auto rot1 = rtl::Rotation3f(rtl::C_PIf / (float)i_max * (float)i, rtl::Vector3f::baseY());
                auto rot2 = rtl::Rotation3f(2.0f * rtl::C_PIf / (float)i_max * (float)i, rtl::Vector3f(1, 1, 1));
                le3.setView(75, rtl::RigidTf3f(rot2(rot1(rot0)), rtl::Translation3f (0, 0, -2)));
                return le3;
            };

    ld.addGridLE(rot_view_lambda_const_dist, 4, 20, "Rotating cube with edges and vertices marked out.");

    return 0;
}