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

#include <cstdio>
#include <cmath>
#include <ctime>
#include <iostream>
#include <utility>
#include <vector>
#include <string>
#include <chrono>
#include <random>

#include "rtl/Core.h"
#include "rtl/Vectorization.h"
#include "rtl/io/LaTeXDoc.h"

struct BenchmarkItem2D
{
    size_t repeat;
    float sigma, delta, dp_time, rw_time, itls_t_time, ftls_t_time, aftls_t_time;
    std::vector<rtl::Vector2f> points;
    std::vector<rtl::LineSegment2f> dp_lines{}, rw_lines{}, itls_t_lines{}, ftls_t_lines{}, aftls_t_lines{};

    static const size_t print_w_pt = 12, print_w_vec = 20;

    BenchmarkItem2D(std::vector<rtl::Vector2f> points, size_t repeat, float sigma, float delta)
    {
        this->points = std::move(points);
        this->repeat = repeat;
        this->sigma = sigma;
        this->delta = delta;
        dp_time = rw_time = itls_t_time = ftls_t_time = aftls_t_time = 0.0f;
    }

    void print(size_t extra_repeat = 1)
    {
        auto field_gen = [this, &extra_repeat](float time, const std::vector<rtl::LineSegment2f> &lines)
        {
            std::string str = std::to_string(time * 1000000 / repeat / extra_repeat);
            str.append(" (");
            str.append(std::to_string(lines.size()));
            str.append(")");
            str.append(print_w_vec - str.size(), ' ');
            return str;
        };
        std::string pt_nr = std::to_string(points.size());
        pt_nr.append(print_w_pt - pt_nr.size(), ' ');
        std::string dp_print = field_gen(dp_time, dp_lines);
        std::string rw_print = field_gen(rw_time, rw_lines);
        std::string itls_t_print = field_gen(itls_t_time, itls_t_lines);
        std::string ftls_t_print = field_gen(ftls_t_time, ftls_t_lines);
        std::string aftls_t_print = field_gen(aftls_t_time, aftls_t_lines);

        std::cout << pt_nr << rw_print << dp_print  << itls_t_print << ftls_t_print << aftls_t_print << std::endl;
    }

    void exportLaTeX(rtl::LaTeXTikz2D &le)
    {
        le.addGridH("style={draw=gray,dotted,thin}", 2);
        le.addGridV("style={draw=gray,dotted,thin}", 2);

        if (points.size() < 1000)
            le.addPlot(points, "style={draw=none}", "style={draw=black,fill=black}", rtl::LaTeXTikz2D::latex_mark_dot, 0.2f);
        else
        {
            size_t step = points.size() / 1000;
            std::vector<rtl::Vector2f> decimated;
            decimated.reserve(2000);
            for (size_t i = 0; i < points.size(); i += step)
                decimated.push_back(points[i]);
            le.addPlot(decimated, "style={draw=none}", "style={draw=black,fill=black}", rtl::LaTeXTikz2D::latex_mark_dot, 0.2f);
        }

        le.addEdges(rw_lines, "style={draw=black,line cap=round,thick}");
        le.addEdges(dp_lines, "style={draw=gray,line cap=round,thick}");
        le.addEdges(ftls_t_lines, "style={draw=blue,line cap=round,thick}");
        le.addEdges(aftls_t_lines, "style={draw=red,line cap=round,thick}");
    }

    static void printHeader()
    {
        auto fieldgen = [](std::string str, size_t len) { return str.append(len - str.size(), ' '); };

        std::string pt_nr = fieldgen("Point nr.", print_w_pt);
        std::string dp_head = fieldgen("Douglas-Peucker", print_w_vec);
        std::string rw_head = fieldgen("Reumean-Witkam", print_w_vec);
        std::string itls_t_head = fieldgen("Templated ITLS", print_w_vec);
        std::string ftls_t_head = fieldgen("Templated FTLS", print_w_vec);
        std::string aftls_t_head = fieldgen("Templated AFTLS", print_w_vec);

        std::cout << pt_nr << rw_head << dp_head << itls_t_head << ftls_t_head << aftls_t_head << std::endl;
    }
};

struct BenchmarkRunner2D
{
    void runBenchmark(BenchmarkItem2D &bi, bool equalize, size_t extra_repeat = 1)
    {
        clock_t clk;

        dp_vec.setEpsilon(3.0f * bi.sigma);
        clk = clock();
        for(size_t j = 0; j < (equalize ? (bi.repeat * extra_repeat) : 1); j++)
            dp_vec(bi.points, bi.dp_lines);
        clk = clock() - clk;
        bi.dp_time = (float)clk / CLOCKS_PER_SEC;

        rw_vec.setEpsilon(3.0f * bi.sigma);
        clk = clock();
        for(size_t j = 0; j < (equalize ? (bi.repeat * extra_repeat) : 1); j++)
            rw_vec(bi.points, bi.rw_lines);
        clk = clock() - clk;
        bi.rw_time = (float)clk / CLOCKS_PER_SEC;

        itls_vec_t.setSigma(bi.sigma);
        clk = clock();
        for (size_t j = 0; j < (equalize ? (bi.repeat * extra_repeat) : 1); j++)
        {
            itls_vec_t(bi.points);
            bi.itls_t_lines = itls_vec_t.lineSegments();
        }
        clk = clock() - clk;
        bi.itls_t_time = (float)clk / CLOCKS_PER_SEC;

        ftls_vec_t.setSigma(bi.sigma);
        ftls_vec_t.setDelta(bi.delta);
        ftls_vec_t.setMaxSize(bi.points.size());
        clk = clock();
        for (size_t j = 0; j < (equalize ? (bi.repeat * extra_repeat) : 1); j++)
        {
            ftls_vec_t(bi.points);
            bi.ftls_t_lines = ftls_vec_t.lineSegments();
        }
        clk = clock() - clk;
        bi.ftls_t_time = (float)clk / CLOCKS_PER_SEC;

        aftls_vec_t.setSigma(bi.sigma);
        aftls_vec_t.setDelta(bi.delta);
        aftls_vec_t.setMaxSize(bi.points.size());
        aftls_vec_t.setSimplexShift(1 + bi.points.size() / 1000);
        clk = clock();
        for (size_t j = 0; j < (equalize ? (bi.repeat * extra_repeat) : 1); j++)
        {
            aftls_vec_t(bi.points);
            bi.aftls_t_lines = aftls_vec_t.lineSegments();
        }
        clk = clock() - clk;
        bi.aftls_t_time = (float)clk / CLOCKS_PER_SEC;
    }

    rtl::VectorizerDouglasPeucker2f dp_vec;
    rtl::VectorizerReumannWitkam2f rw_vec;
    rtl::VectorizerFTLSPolyline2D<float, double> ftls_vec_t;
    rtl::VectorizerAFTLSPolyline2D<float, double> aftls_vec_t;
    rtl::VectorizerITLSProjections2D<float, double> itls_vec_t;
};

struct BenchmarkItem3D
{
    size_t repeat;
    float sigma, itls_t_time, ftls_t_time, aftls_t_time;
    std::vector<rtl::Vector3f> points;
    std::vector<rtl::LineSegment3f> itls_t_lines{}, ftls_t_lines{}, aftls_t_lines{};

    static const size_t print_w_pt = 12, print_w_vec = 20;

    BenchmarkItem3D(std::vector<rtl::Vector3f> points, size_t repeat, float sigma)
    {
        this->points = std::move(points);
        this->repeat = repeat;
        this->sigma = sigma;
        itls_t_time = ftls_t_time = aftls_t_time = 0.0f;
    }

    void print(size_t extra_repeat = 1)
    {
        auto field_gen = [this, &extra_repeat](float time, const std::vector<rtl::LineSegment3f> &lines)
        {
            std::string str = std::to_string(time * 1000000 / repeat / extra_repeat);
            str.append(" (");
            str.append(std::to_string(lines.size()));
            str.append(")");
            str.append(print_w_vec - str.size(), ' ');
            return str;
        };
        std::string pt_nr = std::to_string(points.size());
        pt_nr.append(print_w_pt - pt_nr.size(), ' ');
        std::string itls_t_print = field_gen(itls_t_time, itls_t_lines);
        std::string ftls_t_print = field_gen(ftls_t_time, ftls_t_lines);
        std::string aftls_t_print = field_gen(aftls_t_time, aftls_t_lines);

        std::cout << pt_nr << itls_t_print << ftls_t_print << aftls_t_print << std::endl;
    }

    void exportLaTeX(rtl::LaTeXTikz3D &le)
    {
        le.setView(45, rtl::Transformation3f(-rtl::C_PI / 4.0, rtl::Vector3f::baseX(), rtl::Vector3f::nan()));
        if (points.size() < 1000)
            le.addMarks(points, "style={draw=black,fill=black}", rtl::LaTeXTikz3D::latex_mark_dot, 0.0f, 0.02f);
        else
        {
            size_t step = points.size() / 1000;
            std::vector<rtl::Vector3f> decimated;
            decimated.reserve(2000);
            for (size_t i = 0; i < points.size(); i += step)
                decimated.push_back(points[i]);
            le.addMarks(decimated, "style={draw=black,fill=black}", rtl::LaTeXTikz3D::latex_mark_dot, 0.0f, 0.02f);
        }

        le.addLines(ftls_t_lines, "style={draw=blue,line cap=round,thick}");
        le.addLines(aftls_t_lines, "style={draw=red,line cap=round,thick}");
    }

    static void printHeader()
    {
        auto fieldgen = [](std::string str, size_t len) { return str.append(len - str.size(), ' '); };

        std::string pt_nr = fieldgen("Point nr.", print_w_pt);
        std::string itls_t_head = fieldgen("Templated ITLS", print_w_vec);
        std::string ftls_t_head = fieldgen("Templated FTLS", print_w_vec);
        std::string aftls_t_head = fieldgen("Templated AFTLS", print_w_vec);

        std::cout << pt_nr << itls_t_head << ftls_t_head << aftls_t_head << std::endl;
    }
};

struct BenchmarkRunner3D
{
    void runBenchmark(BenchmarkItem3D &bi, bool equalize, size_t extra_repeat = 1)
    {
        clock_t clk;

        itls_vec_t.setSigma(bi.sigma);
        clk = clock();
        for (size_t j = 0; j < (equalize ? (bi.repeat * extra_repeat) : 1); j++)
        {
            itls_vec_t(bi.points);
            bi.itls_t_lines = itls_vec_t.lineSegments();
        }
        clk = clock() - clk;
        bi.itls_t_time = (float)clk / CLOCKS_PER_SEC;

        ftls_vec_t.setSigma(bi.sigma);
        ftls_vec_t.setMaxSize(bi.points.size());
        clk = clock();
        for (size_t j = 0; j < (equalize ? (bi.repeat * extra_repeat) : 1); j++)
        {
            ftls_vec_t(bi.points);
            bi.ftls_t_lines = ftls_vec_t.lineSegments();
        }
        clk = clock() - clk;
        bi.ftls_t_time = (float)clk / CLOCKS_PER_SEC;

        aftls_vec_t.setSigma(bi.sigma);
        aftls_vec_t.setMaxSize(bi.points.size());
        aftls_vec_t.setSimplexShift(1 + bi.points.size() / 1000);
        clk = clock();
        for (size_t j = 0; j < (equalize ? (bi.repeat * extra_repeat) : 1); j++)
        {
            aftls_vec_t(bi.points);
            bi.aftls_t_lines = aftls_vec_t.lineSegments();
        }
        clk = clock() - clk;
        bi.aftls_t_time = (float)clk / CLOCKS_PER_SEC;
    }

    rtl::VectorizerFTLSProjections3D<float, double> ftls_vec_t;
    rtl::VectorizerAFTLSProjections3D<float, double> aftls_vec_t;
    rtl::VectorizerITLSProjections3D<float, double> itls_vec_t;
};

std::vector<rtl::Vector2f> genHemicycle(int n, float r)
{
    rtl::Vector2f p;
    std::vector<rtl::Vector2f> output;
    for(int i = 0; i < n; i++)
    {
        p.setX(std::cos((float)i * rtl::C_PIf / (float)(n - 1)) * r);
        p.setY(std::sin((float)i * rtl::C_PIf / (float)(n - 1)) * r);
        output.push_back(p);
    }
    return output;
}

std::vector<rtl::Vector2f> genSpikes(int n, int spikes, float height, float width)
{
    rtl::Vector2f p;
    float edge;
    float slope_incr = height * (float)spikes * 2.0f / (float)(n - 1);
    float pt_nr = (float)(n - 1) / (float)(spikes * 2);
    std::vector<rtl::Vector2f>output;

    for(int i = 0; i < n; i++)
    {
        p.setX((float)i * width / (float)(n - 1) - width / 2);
        p.setY((float)i * slope_incr);
        edge = std::floor((float)i / pt_nr);
        if(std::fmod(edge, 2.0f) == 0)
            p.setY(p.y() - height * edge);
        else
            p.setY(-(p.y() - height * (edge + 1)));
        output.push_back(p);
    }
    return output;
}

std::vector<rtl::Vector3f> genSpiral(int n, float radius, float slope, float length)
{
    std::vector<rtl::Vector3f> p;
    float step = length / float(n - 1);
    for (int i = 0; i < n; i++)
    {
        float t = (float)i * step;
        p.emplace_back(radius * std::cos(t), radius * std::sin(t), slope * t);
    }
    return p;
}

std::vector<rtl::Vector3f> genCrown(int n, int spikes, float radius, float height)
{
    std::vector<rtl::Vector3f> p;
    std::vector<rtl::Vector3f> vertices;
    for (int i = 0; i < 2 * spikes + 1; i++)
    {
        float t = 2.0f * rtl::C_PIf * (float)i / (float)(2 * spikes);
        vertices.emplace_back(radius * std::cos(t), radius * std::sin(t), (float)(i % 2) * height);
    }

    for (int i = 0; i < n; i++)
    {
        int segment = (int)((float)i / (float)n * (float)(2 * spikes));
        float t = (float)i / (float)n * (float)(2 * spikes) - (float)segment;
        p.emplace_back((1.0f - t) * vertices[segment] + t * vertices[segment + 1]);
    }
    return p;
}

void benchmark2D(bool equalize, int extra_repeat = 1)
{
    BenchmarkRunner2D br;
    std::vector<BenchmarkItem2D> bis;
    rtl::LaTeXDoc ld("t_vectorization_out", "vect2D_test");

    auto execute_bi_profiler = [&br, &bis, &ld, equalize, extra_repeat](const std::string& desc)
    {
        BenchmarkItem2D::printHeader();
        for (auto &bi : bis)
        {
            br.runBenchmark(bi, equalize, extra_repeat);
            bi.print(extra_repeat);
            rtl::LaTeXTikz2D le;
            bi.exportLaTeX(le);
            ld.addLE(le, desc);
        }
    };

    bis.emplace_back(genHemicycle(10, 8), 100, 0.03f, 3.0f);
    bis.emplace_back(genHemicycle(20, 8), 100, 0.03f, 3.0f);
    bis.emplace_back(genHemicycle(50, 8), 100, 0.03f, 3.0f);
    bis.emplace_back(genHemicycle(100, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genHemicycle(200, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genHemicycle(500, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genHemicycle(1000, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genHemicycle(2000, 8), 1, 0.03f, 3.0f);
    bis.emplace_back(genHemicycle(5000, 8), 1, 0.03f, 3.0f);
    bis.emplace_back(genHemicycle(10000, 8), 1, 0.03f, 3.0f);
    bis.emplace_back(genHemicycle(20000, 8), 1, 0.03f, 3.0f);

    std::string desc = "Hemicycle vectorization";
    std::cout << "\n" << desc << std::endl;
    execute_bi_profiler(desc);

    bis.clear();
    bis.emplace_back(genSpikes(10, 1, 4, 8), 100, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(20, 1, 4, 8), 100, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(50, 1, 4, 8), 100, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(100, 1, 4, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(200, 1, 4, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(500, 1, 4, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(1000, 1, 4, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(2000, 1, 4, 8), 1, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(5000, 1, 4, 8), 1, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(10000, 1, 4, 8), 1, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(20000, 1, 4, 8), 1, 0.03f, 3.0f);

    desc = "Single spike vectorization";
    std::cout << "\n" << desc << std::endl;
    execute_bi_profiler(desc);

    bis.clear();
    bis.emplace_back(genSpikes(20, 5, 4, 8), 100, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(50, 5, 4, 8), 100, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(100, 5, 4, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(200, 5, 4, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(500, 5, 4, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(1000, 5, 4, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(2000, 5, 4, 8), 1, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(5000, 5, 4, 8), 1, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(10000, 5, 4, 8), 1, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(20000, 5, 4, 8), 1, 0.03f, 3.0f);

    desc = "Five spikes vectorization";
    std::cout << "\n" << desc << std::endl;
    execute_bi_profiler(desc);

    bis.clear();
    bis.emplace_back(genSpikes(100, 20, 4, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(200, 20, 4, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(500, 20, 4, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(1000, 20, 4, 8), 10, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(2000, 20, 4, 8), 1, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(5000, 20, 4, 8), 1, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(10000, 20, 4, 8), 1, 0.03f, 3.0f);
    bis.emplace_back(genSpikes(20000, 20, 4, 8), 1, 0.03f, 3.0f);

    desc = "Twenty spikes vectorization";
    std::cout << "\n" << desc << std::endl;
    execute_bi_profiler(desc);
}

void benchmark3D(bool equalize, int extra_repeat = 1)
{
    BenchmarkRunner3D br;
    std::vector<BenchmarkItem3D> bis;
    rtl::LaTeXDoc ld("t_vectorization_out", "vect3D_test");

    auto execute_bi_profiler = [&br, &bis, &ld, equalize, extra_repeat](const std::string& desc)
    {
        BenchmarkItem3D::printHeader();
        for (auto &bi : bis)
        {
            br.runBenchmark(bi, equalize, extra_repeat);
            bi.print(extra_repeat);
            rtl::LaTeXTikz3D le;
            bi.exportLaTeX(le);
            ld.addLE(le, desc);
        }
    };

    bis.emplace_back(genSpiral(10, 8, 0.5, 4.0f * rtl::C_PIf), 100, 0.03f);
    bis.emplace_back(genSpiral(20, 8, 0.5, 4.0f * rtl::C_PIf), 100, 0.03f);
    bis.emplace_back(genSpiral(50, 8, 0.5, 4.0f * rtl::C_PIf), 100, 0.03f);
    bis.emplace_back(genSpiral(100, 8, 0.5, 4.0f * rtl::C_PIf), 10, 0.03f);
    bis.emplace_back(genSpiral(200, 8, 0.5, 4.0f * rtl::C_PIf), 10, 0.03f);
    bis.emplace_back(genSpiral(500, 8, 0.5, 4.0f * rtl::C_PIf), 10, 0.03f);
    bis.emplace_back(genSpiral(1000, 8, 0.5, 4.0f * rtl::C_PIf), 10, 0.03f);
    bis.emplace_back(genSpiral(2000, 8, 0.5, 4.0f * rtl::C_PIf), 1, 0.03f);
    bis.emplace_back(genSpiral(5000, 8, 0.5, 4.0f * rtl::C_PIf), 1, 0.03f);
    bis.emplace_back(genSpiral(10000, 8, 0.5, 4.0f * rtl::C_PIf), 1, 0.03f);
    bis.emplace_back(genSpiral(20000, 8, 0.5, 4.0f * rtl::C_PIf), 1, 0.03f);

    std::string desc = "Spiral vectorization";
    std::cout << "\n" << desc << std::endl;
    execute_bi_profiler(desc);

    bis.clear();
    bis.emplace_back(genCrown(10, 2, 4, 8), 100, 0.03f);
    bis.emplace_back(genCrown(20, 2, 4, 8), 100, 0.03f);
    bis.emplace_back(genCrown(50, 2, 4, 8), 100, 0.03f);
    bis.emplace_back(genCrown(100, 2, 4, 8), 10, 0.03f);
    bis.emplace_back(genCrown(200, 2, 4, 8), 10, 0.03f);
    bis.emplace_back(genCrown(500, 2, 4, 8), 10, 0.03f);
    bis.emplace_back(genCrown(1000, 2, 4, 8), 10, 0.03f);
    bis.emplace_back(genCrown(2000, 2, 4, 8), 1, 0.03f);
    bis.emplace_back(genCrown(5000, 2, 4, 8), 1, 0.03f);
    bis.emplace_back(genCrown(10000, 2, 4, 8), 1, 0.03f);
    bis.emplace_back(genCrown(20000, 2, 4, 8), 1, 0.03f);

    desc = "Two spike crown vectorization";
    std::cout << "\n" << desc << std::endl;
    execute_bi_profiler(desc);

    bis.clear();
    bis.emplace_back(genCrown(20, 5, 4, 8), 100, 0.03f);
    bis.emplace_back(genCrown(50, 5, 4, 8), 100, 0.03f);
    bis.emplace_back(genCrown(100, 5, 4, 8), 10, 0.03f);
    bis.emplace_back(genCrown(200, 5, 4, 8), 10, 0.03f);
    bis.emplace_back(genCrown(500, 5, 4, 8), 10, 0.03f);
    bis.emplace_back(genCrown(1000, 5, 4, 8), 10, 0.03f);
    bis.emplace_back(genCrown(2000, 5, 4, 8), 1, 0.03f);
    bis.emplace_back(genCrown(5000, 5, 4, 8), 1, 0.03f);
    bis.emplace_back(genCrown(10000, 5, 4, 8), 1, 0.03f);
    bis.emplace_back(genCrown(20000, 5, 4, 8), 1, 0.03f);

    desc = "Five spike crown vectorization";
    std::cout << "\n" << desc << std::endl;
    execute_bi_profiler(desc);

    bis.clear();
    bis.emplace_back(genCrown(100, 20, 4, 8), 10, 0.03f);
    bis.emplace_back(genCrown(200, 20, 4, 8), 10, 0.03f);
    bis.emplace_back(genCrown(500, 20, 4, 8), 10, 0.03f);
    bis.emplace_back(genCrown(1000, 20, 4, 8), 10, 0.03f);
    bis.emplace_back(genCrown(2000, 20, 4, 8), 1, 0.03f);
    bis.emplace_back(genCrown(5000, 20, 4, 8), 1, 0.03f);
    bis.emplace_back(genCrown(10000, 20, 4, 8), 1, 0.03f);
    bis.emplace_back(genCrown(20000, 20, 4, 8), 1, 0.03f);

    desc = "Twenty spike crown vectorization";
    std::cout << "\n" << desc << std::endl;
    execute_bi_profiler(desc);
}

/*
void evaluatePrecision(const std::string &name, unsigned int cluster_cnt)
{
    double error_rw, error_dp, error_tls, error_aftls;
    std::string edges_path = "prec_data\\edges_" + name + ".csv";
    std::string edges_data = "prec_data\\data_" + name;
    int point_cnt[] = { 200, 500, 1000, 2000, 5000, 10000, 20000 };
    float sigma = 3, abs_multiple = 5, delta = 50;

    std::vector<rtl::LineSegment2f> result_rw, result_dp, result_tls, result_aftls, environment, ideal, result_tmp;
    std::vector<rtl::Vector2f> points;
    rtl::Observer2D<>::expectedView(environment, rtl::Pose2Df(0, 0, 0), ideal);
    rtl::FTLS_Vectorizer vect_ftls(20000, sigma, delta);
    rtl::AFTLS_Vectorizer vect_aftls(20000, sigma, delta, 1);

    environment = CSV_Import::loadLineSegment2D<float>(edges_path);

    std::ofstream ofs;
    ofs.open("prec_" + name + ".csv", std::ofstream::out | std::ofstream::trunc);

    for (unsigned int i = 0; i < 7; i++)
    {
        std::string edges_plan = edges_data + "\\data_" + name + "_plan_" + std::to_string(i);
        error_rw = error_dp = error_tls = error_aftls = 0;

        for (unsigned int j = 0; j < 1000; j++)
        {
            std::string edges_experiemnt = edges_plan + "\\data_" + name + "_" + std::to_string(j);
            result_rw.clear();
            result_dp.clear();
            result_tls.clear();
            result_aftls.clear();
            for (unsigned int k = 0; k < cluster_cnt; k++)
            {
                points = CSV_Import::loadVector2D<float>(edges_experiemnt + "\\" + std::to_string(k) + std::string(".csv"));
                if (points.empty())
                    continue;
                vectRW(points, result_tmp, abs_multiple * sigma);
                result_rw.insert(result_rw.end(), result_tmp.begin(), result_tmp.end());
                vectDP(points, result_tmp, abs_multiple * sigma);
                result_dp.insert(result_dp.end(), result_tmp.begin(), result_tmp.end());
                vect_ftls(points, result_tmp);
                result_tls.insert(result_tls.end(), result_tmp.begin(), result_tmp.end());
                vect_aftls(points, result_tmp);
                result_aftls.insert(result_aftls.end(), result_tmp.begin(), result_tmp.end());
            }
            error_rw += rtl::Observer2D<>::scanDiff(result_rw, ideal);
            error_dp += rtl::Observer2D<>::scanDiff(result_dp, ideal);
            error_tls += rtl::Observer2D<>::scanDiff(result_tls, ideal);
            error_aftls += rtl::Observer2D<>::scanDiff(result_aftls, ideal);
        }
        ofs << point_cnt[i] << "," << error_rw / 1000 << "," << error_dp / 1000 << "," << error_tls / 1000 << "," << error_aftls / 1000 << "\n";
    }
    ofs.close();
}

void testRealScans(std::string name, float true_lines)
{
    std::vector<rtl::Vector2f> pts;
    std::vector<rtl::LineSegment2f> result;
    pts = CSV_Import::loadVector2D<float>(name + ".csv");
    rtl::CAR_Segmenter segmenter(10, 0.05, 0.2, 2000);
    float sigma = 0.02;
    rtl::INC_Vectorizer inc_vec(2000, sigma, 0.1);
    rtl::FTLS_Vectorizer ftls_vec(2000, sigma, 0.1);
    rtl::AFTLS_Vectorizer aftls_vec(2000, sigma, 0.1, 1);
    rtl::Observation2D obs;
    obs.loadPointCloud(pts);
    obs.processPointCloud(segmenter, aftls_vec, 15);
    unsigned int repeat = 10;
    clock_t clk;
    BenchmarkItem2D bi(2000, 1, 1), bi2(2000, 1, 1);

    std::cout << "Data set: " << name << std::endl;
    std::cout << "Total points: " << pts.sum_nr() << std::endl;

    for (unsigned int j = 0; j < obs.edges().sum_nr(); j++)
    {
        pts = obs.edges()[j].points();

        clk = clock();
        for (unsigned int j = 0; j < repeat; j++)
            vectDP(pts, result, 3 * sigma);
        clk = clock() - clk;
        bi.dp_time += (float)clk / CLOCKS_PER_SEC;

        clk = clock();
        for (unsigned int j = 0; j < repeat; j++)
            vectRW(pts, result, 3 * sigma);
        clk = clock() - clk;
        bi.rw_time += (float)clk / CLOCKS_PER_SEC;

        clk = clock();
        for (unsigned int j = 0; j < repeat; j++)
            inc_vec(pts, result);
        clk = clock() - clk;
        bi.inc_time += (float)clk / CLOCKS_PER_SEC;

        clk = clock();
        for (unsigned int j = 0; j < repeat; j++)
            ftls_vec(pts, result);
        clk = clock() - clk;
        bi.ftls_time += (float)clk / CLOCKS_PER_SEC;

        clk = clock();
        for (unsigned int j = 0; j < repeat; j++)
            aftls_vec(pts, result);
        clk = clock() - clk;
        bi.aftls_time += (float)clk / CLOCKS_PER_SEC;
    }

    std::cout << "DP time:" << bi.dp_time << " " << (bi.dp_time - bi.ftls_time) / bi.ftls_time * 100.0 << std::endl;
    std::cout << "RW time:" << bi.rw_time << " " << (bi.rw_time - bi.ftls_time) / bi.ftls_time * 100.0 << std::endl;
    std::cout << "INC time:" << bi.inc_time << " " << (bi.inc_time - bi.ftls_time) / bi.ftls_time * 100.0 << std::endl;
    std::cout << "FTLS time:" << bi.ftls_time << " " << (bi.ftls_time - bi.ftls_time) / bi.ftls_time * 100.0 << std::endl;
    std::cout << "AFTLS time:" << bi.aftls_time << " " << (bi.aftls_time - bi.ftls_time) / bi.ftls_time * 100.0 << std::endl;
    std::cout << std::endl;

    for (unsigned int j = 0; j < obs.edges().sum_nr(); j++)
    {
        pts = obs.edges()[j].points();

        vectDP(pts, result, 3 * sigma);
        bi2.dp_time += result.sum_nr();

        vectRW(pts, result, 3 * sigma);
        bi2.rw_time += result.sum_nr();

        inc_vec(pts, result);
        bi2.inc_time += result.sum_nr();

        ftls_vec(pts, result);
        bi2.ftls_time += result.sum_nr();

        aftls_vec(pts, result);
        bi2.aftls_time += result.sum_nr();
    }

    std::cout << "DP lines:" << bi2.dp_time << " " << (bi2.dp_time - true_lines) / true_lines * 100.0 << std::endl;
    std::cout << "RW lines:" << bi2.rw_time << " " << (bi2.rw_time - true_lines) / true_lines * 100.0 << std::endl;
    std::cout << "INC lines:" << bi2.inc_time << " " << (bi2.inc_time - true_lines) / true_lines * 100.0 << std::endl;
    std::cout << "FTLS lines:" << bi2.ftls_time << " " << (bi2.ftls_time - true_lines) / true_lines * 100.0 << std::endl;
    std::cout << "AFTLS lines:" << bi2.aftls_time << " " << (bi2.aftls_time - true_lines) / true_lines * 100.0 << std::endl;

    std::cout << "\n" << std::endl;
}
*/

template <typename Element, typename Compute>
void tlsPrecomputedArray()
{
    std::vector<rtl::Vector2D<Element>> vec;
    size_t len = 10;

    for (size_t i = 0; i < len; i++)
        vec.emplace_back(1, 2);

    rtl::PrecArray2D<Element, Compute> arr;
    arr.precompute(vec);
    std::cout<<"\nPrecomputed sums for ten Vector2D(1,2):"<<std::endl;
    for (size_t i = 0; i < len+1; i++)
        std::cout<<arr.array(i, 0)<<"\t"<<arr.array(i, 1)<<"\t"<<arr.array(i, 2)<<"\t"<<arr.array(i, 3)<<"\t"<<arr.array(i, 4)<<"\t"<<i<<std::endl;

    std::cout<<"\nThe same with sums(i):"<<std::endl;
    for (size_t i = 0; i < len+1; i++)
    {
        auto sums = arr.sums(i);
        std::cout<<sums.sx()<<"\t"<<sums.sy()<<"\t"<<sums.sx2()<<"\t"<<sums.sy2()<<"\t"<<sums.sxy()<<"\t"<<sums.cnt()<<std::endl;
    }

    std::cout<<"\nNeighbour sum diff with sums(beg, end):"<<std::endl;
    for (size_t i = 1; i < len+1; i++)
    {
        auto sums = arr.sums(i-1, i);
        std::cout<<sums.sx()<<"\t"<<sums.sy()<<"\t"<<sums.sx2()<<"\t"<<sums.sy2()<<"\t"<<sums.sxy()<<"\t"<<sums.cnt()<<std::endl;
    }

}

template <typename Element, typename Compute>
void tlsLine2D(size_t repeat, size_t point_nr, Element epsilon)
{
    std::cout<<"\nTLS approximation of Vector2D array:"<<std::endl;
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<Element> rnd_element(-1, 1);
    auto el_rnd_gen = [&generator, &rnd_element](){ return rnd_element(generator); };

    for (size_t i = 0; i < repeat; i++)
    {
        std::vector<rtl::Vector2D<Element>> vec;
        rtl::Vector2D<Element> displacement = rtl::Vector2D<Element>::random(el_rnd_gen);
        rtl::Vector2D<Element> direction(-displacement.y(), displacement.x());
        direction *= el_rnd_gen() / 10.0;
        for (size_t j = 0; j < point_nr; j++)
            vec.emplace_back(displacement + direction * j);

        rtl::PrecArray2D<Element, Compute> arr;
        arr.precompute(vec);
        rtl::ApproximationTlsLine2D<Element, Compute> tls_al;
        tls_al(arr.sums(point_nr));
        direction.normalize();
        if (std::abs(std::abs(direction.x()) - std::abs(tls_al.direction().x())) < epsilon &&
            std::abs(std::abs(direction.y()) - std::abs(tls_al.direction().y())) < epsilon &&
                std::abs(displacement.length() - std::abs(tls_al.c())) < epsilon)
            continue;

        std::cout<<"\tdirection: "<<direction.x()<<", "<<direction.y()<<"\tdist: "<<displacement.length()<<"\terr: 0"<<std::endl;
        std::cout << "\tdirection: " << tls_al.direction().x() << ", " << tls_al.direction().y() << "\tdist: " << tls_al.c() << "\terr: " << tls_al.errSquared() << std::endl;
    }
}

int main()
{
    /*genHemicycle(pts, 200, 8);
    ofs.open("hemicycle.csv", std::ofstream::out | std::ofstream::trunc);
    for (int i = 0; i < 200; i++)
        ofs<<pts[i].x<<","<<pts[i].y<<"\n";
    ofs.close();*/

    //genSpikes(pts, 1000, 5, 5, 16);

    //AFTLS_Vectorizer vec(20000, sigma, delta, 1);
    //vec(pts, result);

    /*std::cout<<"Points:"<<std::endl;
    for(unsigned int i = 0; i < pts.sum_nr(); i++)
        std::cout<<i<<"\t"<<pts[i].x<<"\t"<<pts[i].y<<std::endl;*/

    /*std::cout<<"Results:"<<std::endl;
    for(unsigned int i = 0; i < result.sum_nr(); i++)
        std::cout<<i<<"\t"<<result[i].beg().x()<<"\t"<<result[i].beg().y()<<std::endl;*/

    //benchmark(100);
    //benchmark_stage_time();
    //benchmark_real(1000000);
    //benchmark_synth(300);

    //std::cout<<dummy_int<<std::endl;

    /*Pose2D view_point(1, 0, 0);
    LineSegment2D::loadCSV("edges_room1.csv", environment);
    LS2D_Observer::expScanEnvironment(environment, Pose2D(0, 0, 0), result);
    LS2D_Observer::expScanEnvironment(environment, Pose2D(0, 0, 0), tmp);
    std::cout << "Scan diff area: " << LS2D_Observer::scanDiff(result, tmp) << std::endl;
    poses.push_back(Pose2D(0, 0, 0));
    LaTeXTikz2D le;
    le.addGridH("style={draw=gray,dotted,thin}", 50);
    le.addGridV("style={draw=gray,dotted,thin}", 50);
    le.addEdges(environment, "style={draw=black,line cap=round,ultra thick}");
    le.addEdges(result, "style={draw=red,line cap=round,ultra thick}");
    le.addPoses(poses, "style={draw=black,fill=white,line cap=round,thick}", "style={}", LaTeXTikz2D::export_path | LaTeXTikz2D::export_points);
    le.writeTEX("obs.tex");*/

    /*LineSegment2D::loadCSV("edges_room1.csv", environment);
    LaTeXTikz2D le;
    le.addGridH("style={draw=gray,dotted,thin}", 50);
    le.addGridV("style={draw=gray,dotted,thin}", 50);
    le.addEdges(environment, "style={draw=black,line cap=round,ultra thick}");
    le.writeTEX("obs.tex");*/

    //evaluatePrecision("breaks", 4);
    //evaluatePrecision("cut_square", 1);
    //evaluatePrecision("room1", 8);

    /*testRealScans("corridor_c", 10);
    testRealScans("lab_c", 50);
    testRealScans("empty_room_c", 24);*/

    bool equalize = false;
    benchmark2D(equalize);
    benchmark3D(equalize);

    //plot(genSpiral(1000, 8, 0.5, 4.0f * rtl::C_PIf), "pts.tex");
    //plot(genCrown(1000, 5, 4, 8), "pts.tex");

    size_t repeat = 10;
    float errf = 0.0001f;

    tlsPrecomputedArray<float, double >();
    tlsLine2D<float, double>(repeat, 100, errf);

    std::cout<<"\nClocks per second: " << CLOCKS_PER_SEC << std::endl;
    std::cout<<"\nHigh res clocks per second: " << std::chrono::high_resolution_clock::period::den/std::chrono::high_resolution_clock::period::num<<std::endl;
    std::cout<<"Benchmark complete..."<<std::endl;
    return 0;
}

