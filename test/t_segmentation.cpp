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
#include <chrono>
#include <random>

#include "rtl/Core.h"
#include "rtl/seg/CAR_Segmenter.h"
#include "rtl/seg/IA_Segmenter.h"
#include "rtl/io/LaTeXDoc.h"

template<typename T>
std::vector<rtl::Vector2D<T>> genStepCycle(int n, T radius, T step_prob)
{
    std::vector<rtl::Vector2D<T>> output;
    output.reserve(n);

    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<T> rnd_element(0, 1);

    T r = radius;

    for(int i = 0; i < n; i++)
    {
        if (rnd_element(generator) < step_prob)
            r = radius * 2 * rnd_element(generator);
        output.emplace_back(std::cos((T) i * 2 * rtl::C_PI<T> / (T) (n - 1)) * r, std::sin((T) i * 2 * rtl::C_PI<T> / (T) (n - 1)) * r);
    }
    return output;
}

template<typename T>
std::vector<rtl::Vector3D<T>> genStepSpiral(int n, T radius, T slope, T length, T step_prob)
{
    std::vector<rtl::Vector3D<T>> output;
    output.reserve(n);
    T step = length / T(n - 1);

    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<T> rnd_element(0, 1);

    T r = radius;

    for (int i = 0; i < n; i++)
    {
        if (rnd_element(generator) < step_prob)
            r = radius * 2 * rnd_element(generator);
        T t = (T)i * step;
        output.emplace_back(r * std::cos(t), r * std::sin(t), slope * t);
    }
    return output;
}

void testCAR_Segmenter2D(const std::vector<rtl::Vector2f> &points, rtl::LaTeXTikz2D &le)
{
    rtl::CAR_Segmenter<rtl::Vector2f> seg(10, 0.01, 0.1);
    seg.loadData(points);

    le.clearAll();
    le.addGridH("style={draw=gray,dotted,thin}", 0.5);
    le.addGridV("style={draw=gray,dotted,thin}", 0.5);
    le.setMinPlotRegion(-2.0f, -2.0f, 2.0f, 2.0f);

    float hue = 0.0f, tmp;
    while (seg.clustersAvailable() > 0)
    {
        std::string color = le.saveColor("{hsb}{" + std::to_string(hue) + ",0.8,0.5}");
        std::string style = "style={draw=" + color + ",fill=" + color + "}";
        auto cl = seg.grabCluster();
        le.addPlot(cl, "", style, rtl::LaTeXTikz2D::latex_mark_dot, 0.2f);
        le.addLine(rtl::Vector2f::zeros(), cl.front(), "style={draw=black,ultra thin}");
        le.addLine(rtl::Vector2f::zeros(), cl.back(), "style={draw=black,ultra thin}");
        hue = std::modf(hue + 0.3f, &tmp);
    }
}

void testCAR_Segmenter3D(std::vector<rtl::Vector3f> points, rtl::LaTeXTikz3D &le)
{
    rtl::CAR_Segmenter<rtl::Vector3f> seg(10, 0.01, 0.1);
    seg.loadData(points);

    le.clearAll();
    rtl::RigidTf3f view_tr = rtl::RigidTf3f::identity();
    view_tr.setAngleAxis(rtl::C_PIf / 2.0, rtl::Vector3f(1, 1, 1));
    view_tr.setTrVecX(-2.0);
    view_tr.setTrVecZ(-5.0);
    le.setExportSize(10, 10);
    le.setView(75, view_tr);
    le.setMinPlotRegion(rtl::Vector3f(-2.0f, -2.0f, -2.0f), rtl::Vector3f(2.0f, 2.0f, 2.0f));
    le.addLine({rtl::Vector3f(0, 0, points.front().z() - 0.5f), rtl::Vector3f(0, 0, points.back().z() + 0.5f)}, "style={draw=black,thin}");

    float hue = 0.0f, tmp;
    while (seg.clustersAvailable() > 0)
    {
        std::string color = le.saveColor("{hsb}{" + std::to_string(hue) + ",0.8,0.5}");
        std::string style = "style={draw=" + color + ",fill=" + color + "}";
        auto cl = seg.grabCluster();
        le.addMarks(cl, style, rtl::LaTeXTikz3D::latex_mark_dot, 0.0f, 0.005f);
        le.addLine({rtl::Vector3f(0, 0, cl.front().z()), cl.front()}, "style={draw=black,ultra thin}");
        le.addLine({rtl::Vector3f(0, 0, cl.back().z()), cl.back()}, "style={draw=black,ultra thin}");
        hue = std::modf(hue + 0.3f, &tmp);
    }
}

void testIA_Segmenter2D(const std::vector<rtl::Vector2f> &points, rtl::LaTeXTikz2D &le)
{
    rtl::IA_Segmenter<rtl::Vector2f> seg(10, 0.01, 0.1, 2 * rtl::C_PIf * (float)10 / points.size());
    for (auto &p : points)
        seg.addPoint(p);

    le.clearAll();
    le.addGridH("style={draw=gray,dotted,thin}", 0.5);
    le.addGridV("style={draw=gray,dotted,thin}", 0.5);
    le.setMinPlotRegion(-2.0f, -2.0f, 2.0f, 2.0f);

    float hue = 0.0f, tmp;
    while (seg.closedClustersAvailable() > 0)
    {
        std::string color = le.saveColor("{hsb}{" + std::to_string(hue) + ",0.8,0.5}");
        std::string style = "style={draw=" + color + ",fill=" + color + "}";
        auto cl = seg.grabCluster();
        le.addPlot(cl, "", style, rtl::LaTeXTikz2D::latex_mark_dot, 0.2f);
        le.addLine(rtl::Vector2f::zeros(), cl.front(), "style={draw=black,ultra thin}");
        le.addLine(rtl::Vector2f::zeros(), cl.back(), "style={draw=black,ultra thin}");
        hue = std::modf(hue + 0.3f, &tmp);
    }
    for (const auto &cl_it : seg.aliveClusters())
    {
        std::string color = le.saveColor("{hsb}{" + std::to_string(hue) + ",0.8,0.5}");
        std::string style = "style={draw=" + color + ",fill=" + color + "}";
        le.addPlot(cl_it.second, "", style, rtl::LaTeXTikz2D::latex_mark_dot, 0.2f);
        le.addLine(rtl::Vector2f::zeros(), cl_it.second.front(), "style={draw=black,ultra thin}");
        le.addLine(rtl::Vector2f::zeros(), cl_it.second.back(), "style={draw=black,ultra thin}");
        hue = std::modf(hue + 0.3f, &tmp);
    }
}

void testIA_Segmenter3D(std::vector<rtl::Vector3f> points, rtl::LaTeXTikz3D &le)
{
    rtl::IA_Segmenter<rtl::Vector3f > seg(10, 0.01, 0.1, rtl::C_PIf * (float)10 / points.size());
    for (auto &p : points)
        seg.addPoint(p);

    le.clearAll();
    rtl::RigidTf3f view_tr = rtl::RigidTf3f ::identity();
    view_tr.setAngleAxis(rtl::C_PIf / 2.0, rtl::Vector3f(1, 1, 1));
    view_tr.setTrVecX(-2.0);
    view_tr.setTrVecZ(-5.0);
    le.setExportSize(10, 10);
    le.setView(75, view_tr);
    le.setMinPlotRegion(rtl::Vector3f(-2.0f, -2.0f, -2.0f), rtl::Vector3f(2.0f, 2.0f, 2.0f));
    le.addLine({rtl::Vector3f(0, 0, points.front().z() - 0.5f), rtl::Vector3f(0, 0, points.back().z() + 0.5f)}, "style={draw=black,thin}");

    float hue = 0.0f, tmp;
    while (seg.closedClustersAvailable() > 0)
    {
        std::string color = le.saveColor("{hsb}{" + std::to_string(hue) + ",0.8,0.5}");
        std::string style = "style={draw=" + color + ",fill=" + color + "}";
        auto cl = seg.grabCluster();
        le.addMarks(cl, style, rtl::LaTeXTikz3D::latex_mark_dot, 0.0f, 0.005f);
        le.addLine({rtl::Vector3f(0, 0, cl.front().z()), cl.front()}, "style={draw=black,ultra thin}");
        le.addLine({rtl::Vector3f(0, 0, cl.back().z()), cl.back()}, "style={draw=black,ultra thin}");
        hue = std::modf(hue + 0.3f, &tmp);
    }
    for (const auto &cl_it : seg.aliveClusters())
    {
        std::string color = le.saveColor("{hsb}{" + std::to_string(hue) + ",0.8,0.5}");
        std::string style = "style={draw=" + color + ",fill=" + color + "}";
        le.addMarks(cl_it.second, style, rtl::LaTeXTikz3D::latex_mark_dot, 0.0f, 0.005f);
        le.addLine({rtl::Vector3f(0, 0, cl_it.second.front().z()), cl_it.second.front()}, "style={draw=black,ultra thin}");
        le.addLine({rtl::Vector3f(0, 0, cl_it.second.back().z()), cl_it.second.back()}, "style={draw=black,ultra thin}");
        hue = std::modf(hue + 0.3f, &tmp);
    }
}

int main()
{
    rtl::LaTeXDoc ld("t_segmentation_out", "seg_test");
    ld.setRemoveTmpDir([](const std::string &){ return true; });

    rtl::LaTeXTikz2D le2d;
    rtl::LaTeXTikz3D le3d;

    testCAR_Segmenter2D(genStepCycle<float>(1000, 1.0, 0.01), le2d);
    ld.addLE(le2d, "CAR segmenter in 2D.");

    testCAR_Segmenter3D(genStepSpiral<float>(1000, 1.0, 0.2, 5 * rtl::C_PIf, 0.01), le3d);
    ld.addLE(le3d, "CAR segmenter in 3D.");

    testIA_Segmenter2D(genStepCycle<float>(1000, 1.0, 0.01), le2d);
    ld.addLE(le2d, "IA segmenter in 2D.");

    testIA_Segmenter3D(genStepSpiral<float>(1000, 1.0, 0.2, 5 * rtl::C_PIf, 0.01), le3d);
    ld.addLE(le3d, "IA segmenter in 3D.");
}