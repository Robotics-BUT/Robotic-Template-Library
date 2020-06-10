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

#include "rtl/io/LaTeXTikz2D.h"
#include "rtl/Transformation.h"

#include <cmath>
#include <fstream>
#include <cfloat>
#include <cstdio>

namespace rtl
{
    const unsigned int LaTeXTikz2D::overrun_relative = 0;
    const unsigned int LaTeXTikz2D::overrun_absolute = 1;

    const unsigned int LaTeXTikz2D::axis_type_linear = 0;
    const unsigned int LaTeXTikz2D::axis_type_log10 = 1;

    const unsigned int LaTeXTikz2D::position_above = 0x01;
    const unsigned int LaTeXTikz2D::position_below = 0x02;
    const unsigned int LaTeXTikz2D::position_right = 0x04;
    const unsigned int LaTeXTikz2D::position_left = 0x08;

    const char *LaTeXTikz2D::latex_var_max_x = "\\MaxX";
    const char *LaTeXTikz2D::latex_var_min_x = "\\MinX";
    const char *LaTeXTikz2D::latex_var_max_y = "\\MaxY";
    const char *LaTeXTikz2D::latex_var_min_y = "\\MinY";
    const char *LaTeXTikz2D::latex_var_mark_radius_x = "\\MarkGlobalScaleX";
    const char *LaTeXTikz2D::latex_var_mark_radius_y = "\\MarkGlobalScaleY";

    const char *LaTeXTikz2D::latex_mark_blank = "";
    const char *LaTeXTikz2D::latex_mark_cross = "\\draw (-\\MarkRadius,-\\MarkRadius) -- (\\MarkRadius,\\MarkRadius);\n\t\\draw (\\MarkRadius,-\\MarkRadius) -- (-\\MarkRadius, \\MarkRadius);";
    const char *LaTeXTikz2D::latex_mark_mark = "\\draw (0,-\\MarkRadius) -- (0,\\MarkRadius);";
    const char *LaTeXTikz2D::latex_mark_dot = "\\fill (0, 0) circle [radius=\\MarkRadius];\n\t\\draw (0, 0) circle [radius=\\MarkRadius];";
    const char *LaTeXTikz2D::latex_mark_robot = "\\fill (0, 0) circle [radius=\\MarkRadius];\n\t\\draw (0, 0) circle [radius=\\MarkRadius];\n\t\\draw (0, 0) -- (2*\\MarkRadius,0);";

    LaTeXTikz2D::LaTeXTikz2D(unsigned int axis_x, unsigned int axis_y)
    {
        clearAll();
        axis_type_x = axis_x;
        axis_type_y = axis_y;
    }

    void LaTeXTikz2D::clearData()
    {
        styles.clear();
        marks.clear();
        colors.clear();
        render_codes.clear();

        max_x = max_y = -FLT_MAX;
        min_x = min_y = FLT_MAX;
        has_grid_h = has_grid_v = has_axis_x = has_axis_y = false;

        overrun_type = LaTeXTikz2D::overrun_relative;
        overrun_magnitude = 5.0f;
        label_tick_x = label_tick_y = grid_tick_h = grid_tick_v = 0.0f;
        axis_description_x = axis_description_y = axis_num_format_x = axis_num_format_y = axis_num_position_x = axis_num_position_y = "";
    }

    void LaTeXTikz2D::clearAll()
    {
        clearData();

        scale_x = scale_y = 1.0f;
        mark_radius = 0.1f;
        export_width = export_height = 10.0f;
        export_border = 0.1f;
        clip_p1_x = clip_p1_y = clip_p2_x = clip_p2_y = 0.0f;
        is_clipped = false;
    }

    void LaTeXTikz2D::addPlot(const std::vector<float> &x, const std::vector<float> &y, const std::string &line_style, const std::string &mark_style, const std::string &mark, float mark_scale)
    {
        size_t cnt = x.size();
        if (cnt != y.size())
            return;

        std::string code;

        if (!line_style.empty())
        {
            std::string line_style_name = saveStyle(line_style);
            for (size_t i = 0; i < cnt - 1; i++)
                code += "\t\\draw[" + line_style_name + "] (" + std::to_string(addX(x[i])) + "," +
                        std::to_string(addY(y[i])) + ") -- (" + std::to_string(addX(x[i + 1])) + "," +
                        std::to_string(addY(y[i + 1])) + ");\n";
        }

        if (!mark_style.empty() && !mark.empty())
        {
            std::string mark_style_name = saveStyle(mark_style);
            std::string mark_name = saveMark(mark_style_name, mark, mark_scale);

            float rot, rot_2;
            for (size_t i = 0; i < cnt; i++)
            {
                if (cnt == 1)
                    rot = 0.0f;
                else if (i == 0)
                    rot = Vector2f(x[1] - x.front(), y[1] - y.front()).angleFromZero() * 180.0f / rtl::C_PIf;
                else if (i == cnt - 1)
                    rot = Vector2f(x.back() - x[cnt - 2], y.back() - y[cnt - 2]).angleFromZero() * 180.0f / rtl::C_PIf;
                else
                {
                    rot = Vector2f(x[i] - x[i - 1], y[i] - y[i - 1]).angleFromZero();
                    Vector2f v1(x[i] - x[i - 1], y[i] - y[i - 1]), v2(x[i + 1] - x[i], y[i + 1] - y[i]);
                    rot_2 = Vector2f::angleCcw(v1, v2) / 2.0f;
                    rot = (rot + rot_2) * 180.0f / rtl::C_PIf;
                }
                code += "\t\\" + mark_name + "{" + std::to_string(addX(x[i])) + "}{" + std::to_string(addY(y[i])) +
                        "}{" + std::to_string(rot) + "}\n";
            }
        }

        render_codes.push_back(code);
    }

    void LaTeXTikz2D::addPlot(const std::vector<Vector2f> &v, const std::string &line_style, const std::string &mark_style, const std::string &mark, float mark_scale)
    {
        size_t cnt = v.size();
        std::string code;

        if (!line_style.empty())
        {
            std::string line_style_name = saveStyle(line_style);

            for (size_t i = 0; i < cnt - 1; i++)
                code += "\t\\draw[" + line_style_name + "] (" + std::to_string(addX(v[i].x())) + "," +
                        std::to_string(addY(v[i].y())) + ") -- (" + std::to_string(addX(v[i + 1].x())) + "," +
                        std::to_string(addY(v[i + 1].y())) + ");\n";
        }

        if (!mark_style.empty() && !mark.empty())
        {
            std::string mark_style_name = saveStyle(mark_style);
            std::string mark_name = saveMark(mark_style_name, mark, mark_scale);

            float rot, rot_2;
            for (size_t i = 0; i < cnt; i++)
            {
                if (cnt == 1)
                    rot = 0.0f;
                else if (i == 0)
                    rot = Vector2f(v[1].x() - v.front().x(), v[1].y() - v.front().y()).angleFromZero() * 180.0f /
                          rtl::C_PIf;
                else if (i == cnt - 1)
                    rot = Vector2f(v.back().x() - v[cnt - 2].x(), v.back().y() - v[cnt - 2].y()).angleFromZero() *
                          180.0f / rtl::C_PIf;
                else
                {
                    rot = Vector2f(v[i].x() - v[i - 1].x(), v[i].y() - v[i - 1].y()).angleFromZero();
                    Vector2f v1(v[i].x() - v[i - 1].x(), v[i].y() - v[i - 1].y()), v2(v[i + 1].x() - v[i].x(),
                                                                                       v[i + 1].y() - v[i].y());
                    rot_2 = Vector2f::angleCcw(v1, v2) / 2.0f;
                    rot = (rot + rot_2) * 180.0f / rtl::C_PIf;
                }
                code += "\t\\" + mark_name + "{" + std::to_string(addX(v[i].x())) + "}{" +
                        std::to_string(addY(v[i].y())) + "}{" + std::to_string(rot) + "}\n";
            }
        }

        render_codes.push_back(code);
    }

    void LaTeXTikz2D::addTriangle(const Vector2f &a, const Vector2f &b, const Vector2f &c, const std::string &style)
    {
        std::string style_name = saveStyle(style);
        std::string code =
                "\\filldraw[" + style_name + "] (" + std::to_string(addX(a.x())) + "," + std::to_string(addY(a.y())) +
                ") -- (" + std::to_string(addX(b.x())) + "," + std::to_string(addY(b.y())) + ") -- (" +
                std::to_string(addX(c.x())) + "," + std::to_string(addY(c.y())) + ") -- cycle;";
        render_codes.push_back(code);
    }

    void LaTeXTikz2D::addRectangle(const Vector2f &p1, const Vector2f &p2, const std::string &style)
    {
        std::string style_name = saveStyle(style);
        std::string code = "\\filldraw[" + style_name + "] (" + std::to_string(addX(p1.x())) + ", " +
                           std::to_string(addY(p1.y())) + ") rectangle (" + std::to_string(addX(p2.x())) + ", " +
                           std::to_string(addY(p2.y())) + ");";
        render_codes.push_back(code);
    }

    void LaTeXTikz2D::addQuadrilateral(const Vector2f &a, const Vector2f &b, const Vector2f &c, const Vector2f &d, const std::string &style)
    {
        std::string style_name = saveStyle(style);
        std::string code =
                "\\filldraw[" + style_name + "] (" + std::to_string(addX(a.x())) + "," + std::to_string(addY(a.y())) +
                ") -- (" + std::to_string(addX(b.x())) + "," + std::to_string(addY(b.y())) + ") -- (" +
                std::to_string(addX(c.x())) + "," + std::to_string(addY(c.y())) + ") -- (" +
                std::to_string(addX(d.x())) + "," + std::to_string(addY(d.y())) + ") -- cycle;";
        render_codes.push_back(code);
    }

    void LaTeXTikz2D::addCircle(const Vector2f &centre, float radius, const std::string &style)
    {
        std::string style_name = saveStyle(style);
        std::string code = "\\filldraw[" + style_name + "] (" + std::to_string(addX(centre.x())) + ", " +
                           std::to_string(addY(centre.y())) + ") circle [radius=" + std::to_string(radius) + "];";
        render_codes.push_back(code);
    }

    void LaTeXTikz2D::addEllipse(const Vector2f &centre, float x_radius, float y_radius, float rotation, const std::string &style)
    {
        std::string style_name = saveStyle(style);
        std::string code = "\\filldraw[" + style_name + "] (" + std::to_string(addX(centre.x())) + ", " +
                           std::to_string(addY(centre.y())) + ") circle [x radius=" + std::to_string(x_radius) +
                           ", y radius=" + std::to_string(y_radius) + ", rotate=" +
                           std::to_string(rotation / rtl::C_PIf * 180.0f) + "];";
        render_codes.push_back(code);
    }

    void LaTeXTikz2D::addPie(const Vector2f &centre, float radius, float angle_beg, float angle_end, const std::string &style)
    {
        std::string style_name = saveStyle(style);
        Vector2f arc_beg(radius, 0.0f);
        Rotation2f rot(angle_beg);
        arc_beg = centre + rot(arc_beg);
        std::string code = "\\filldraw[" + style_name + "] (" + std::to_string(addX(centre.x())) + ", " +
                           std::to_string(addY(centre.y())) + ") -- (" + std::to_string(addX(arc_beg.x())) + "," +
                           std::to_string(addY(arc_beg.y())) + ") arc (" +
                           std::to_string(angle_beg / rtl::C_PIf * 180.0f) + ":" +
                           std::to_string(angle_end / rtl::C_PIf * 180.0f) + ":" + std::to_string(radius) +
                           ") -- cycle;";
        render_codes.push_back(code);
    }

    void LaTeXTikz2D::addLine(const Vector2f &beg, const Vector2f &end, const std::string &style)
    {
        std::string style_name = saveStyle(style);
        std::string code =
                "\\draw[" + style_name + "] (" + std::to_string(addX(beg.x())) + ", " + std::to_string(addY(beg.y())) +
                ") -- (" + std::to_string(addX(end.x())) + ", " + std::to_string(addY(end.y())) + ");";
        render_codes.push_back(code);
    }

    void LaTeXTikz2D::addLine(const LineSegment2f &ls, const std::string &style)
    {
        addLine(ls.beg(), ls.end(), style);
    }

    void LaTeXTikz2D::addLines(const std::vector<LineSegment2f> &ls, const std::string &style)
    {
        for (auto &l : ls)
            addLine(l.beg(), l.end(), style);
    }

    void LaTeXTikz2D::addText(const std::string &text, const std::string & style, const Vector2f &position)
    {
        std::string style_name = saveStyle(style);
        std::string code = "\\node[" + style_name + "] at (" + std::to_string(addX(position.x())) + "," +
                           std::to_string(addY(position.y())) + ") {" + text + "};";
        render_codes.push_back(code);
    }

    float LaTeXTikz2D::addX(float x)
    {
        if (axis_type_x == LaTeXTikz2D::axis_type_log10)
            x = log10f(x);
        //x /= 10;
        if (x < min_x)
            min_x = x;
        if (x > max_x)
            max_x = x;
        return x;
    }

    float LaTeXTikz2D::addY(float y)
    {
        if (axis_type_y == LaTeXTikz2D::axis_type_log10)
            y = log10f(y);
        //y /= 10;
        if (y < min_y)
            min_y = y;
        if (y > max_y)
            max_y = y;
        return y;
    }

    std::string LaTeXTikz2D::saveStyle(const std::string &style)
    {
        auto it = styles.find(style);
        if (it == styles.end())
        {
            std::string style_name = "Style" + digitsToLetters(styles.size());
            styles.insert(std::pair<std::string, std::string>(style, style_name));
            return style_name;
        }
        else
        {
            return it->second;
        }
    }

    std::string LaTeXTikz2D::saveMark(const std::string &style_name, const std::string &mark_template, float scale)
    {
        std::string mark_code = "\n\t\\begin{scope}[style=" + style_name + ",xshift=#1cm,yshift=#2cm,xscale=" +
                                LaTeXTikz2D::latex_var_mark_radius_x
                                + ",yscale=" + LaTeXTikz2D::latex_var_mark_radius_y +
                                ",rotate=#3]\n\t\\pgfmathsetmacro{\\MarkRadius}{" + std::to_string(scale) + "}\n\t" +
                                mark_template + "\n\t\\end{scope}";

        auto it = marks.find(mark_code);
        if (it == marks.end())
        {
            std::string mark_name = "Mark" + digitsToLetters(marks.size());
            marks.insert(std::pair<std::string, std::string>(mark_code, mark_name));
            return mark_name;
        }
        else
        {
            return it->second;
        }
    }

    std::string LaTeXTikz2D::saveColor(const std::string &color)
    {
        auto it = colors.find(color);
        if (it == colors.end())
        {
            std::string color_name = "Color" + digitsToLetters(colors.size());
            colors.insert(std::pair<std::string, std::string>(color, color_name));
            return color_name;
        }
        else
        {
            return it->second;
        }
    }

    std::string LaTeXTikz2D::digitsToLetters(size_t num)
    {
        char buffer[50];
        int l = snprintf(buffer, sizeof(buffer), "%zd", num);
        if (l < 0)
            return std::string();
        for (int i = 0; i < l; i++)
            buffer[i] += 17;
        return std::string(buffer);
    }

    std::string LaTeXTikz2D::positionToAnchor(unsigned int pos)
    {
        std::string str = "anchor=";
        if (pos & LaTeXTikz2D::position_above)
            str += "south ";
        else if (pos & LaTeXTikz2D::position_below)
            str += "north ";
        if (pos & LaTeXTikz2D::position_right)
            str += "west";
        else if (pos & LaTeXTikz2D::position_left)
            str += "east";
        return str;
    }

    void LaTeXTikz2D::writeTEX(const std::string &file_name)
    {
        if (has_axis_x || has_grid_h)
        {
            if (overrun_type == LaTeXTikz2D::overrun_relative)
            {
                float tmp = (max_x - min_x) * overrun_magnitude / 100.0f;
                max_x += tmp;
                min_x -= tmp;
            } else if (overrun_type == LaTeXTikz2D::overrun_absolute)
            {
                max_x += overrun_magnitude;
                min_x -= overrun_magnitude;
            }
        }

        if (has_axis_y || has_grid_v)
        {
            if (overrun_type == LaTeXTikz2D::overrun_relative)
            {
                float tmp = (max_y - min_y) * overrun_magnitude / 100.0f;
                max_y += tmp;
                min_y -= tmp;
            } else if (overrun_type == LaTeXTikz2D::overrun_absolute)
            {
                max_y += overrun_magnitude;
                min_y -= overrun_magnitude;
            }
        }

        float output_scale_x, output_scale_y;
        if (is_clipped)
        {
            output_scale_x = std::fmax(export_width / (abs(clip_p1_x - clip_p2_x)), export_height / (abs(clip_p1_y - clip_p2_y))) * scale_x;
            output_scale_y = std::fmax(export_width / (abs(clip_p1_x - clip_p2_x)), export_height / (abs(clip_p1_y - clip_p2_y))) * scale_y;
        } else
        {
            output_scale_x = std::fmax(export_width / (max_x - min_x), export_height / (max_y - min_y)) * scale_x;
            output_scale_y = std::fmax(export_width / (max_x - min_x), export_height / (max_y - min_y)) * scale_y;
        }

        std::ofstream ofs;
        ofs.open(file_name, std::ofstream::out | std::ofstream::trunc);

        // export head
        ofs << "\\documentclass{minimal}\n"
               "\\usepackage[rgb]{xcolor}\n"
               "\\usepackage{tikz}\n"
               "\\usepackage[active,tightpage]{preview}\n"
               "\\PreviewEnvironment{tikzpicture}\n"
               "\\setlength\\PreviewBorder{" << std::to_string(export_border) << "cm}\n"
                                                                                 "\n"
                                                                                 "\\begin{document}\n"
                                                                                 "\n"
                                                                                 "\\begin{tikzpicture}[\n"
                                                                                 "\txscale = "
            << std::to_string(output_scale_x) << ", yscale = " << std::to_string(output_scale_y) << ",\n";

        // export styles
        if (!styles.empty())
        {
            auto it_last = --styles.end();
            for (auto it = styles.begin(); it != it_last; it++)
                ofs << "\t" << it->second << "/." << it->first << ",\n";
            ofs << "\t" << it_last->second << "/." << it_last->first << "\n";
        }
        ofs << "\t]\n\n";

        // export colors
        for (auto & color : colors)
            ofs << "\t" << "\\definecolor{" + color.second + "}" + color.first << "\n";
        ofs << "\n";

        // export LaTeX variables
        ofs << "\t\\pgfmathsetmacro{" << latex_var_max_x << "}{" << std::to_string(max_x) << "}\n";
        ofs << "\t\\pgfmathsetmacro{" << latex_var_min_x << "}{" << std::to_string(min_x) << "}\n";
        ofs << "\t\\pgfmathsetmacro{" << latex_var_max_y << "}{" << std::to_string(max_y) << "}\n";
        ofs << "\t\\pgfmathsetmacro{" << latex_var_min_y << "}{" << std::to_string(min_y) << "}\n";
        ofs << "\t\\pgfmathsetmacro{" << latex_var_mark_radius_x << "}{" << std::to_string(mark_radius / output_scale_x)
            << "}\n";
        ofs << "\t\\pgfmathsetmacro{" << latex_var_mark_radius_y << "}{" << std::to_string(mark_radius / output_scale_y)
            << "}\n";
        if (!axis_description_x.empty() || !axis_description_y.empty())
        {
            ofs << "\t\\newdimen\\XCoord\n";
            ofs << "\t\\newdimen\\YCoord\n";
        }
        ofs << "\n";

        // export marks
        for (auto & mark : marks)
            ofs << "\t" << "\\newcommand{\\" + mark.second + "}[3]{" << mark.first << "\n\t}\n";
        ofs << "\n";

        // export clip region
        if (is_clipped)
            ofs << "\t\\clip (" + std::to_string(clip_p1_x) + "," + std::to_string(clip_p1_y) + ") rectangle (" +
                   std::to_string(clip_p2_x) + "," + std::to_string(clip_p2_y) + ");\n\n";

        // export grids
        if (has_grid_v)
        {
            if (axis_type_x == LaTeXTikz2D::axis_type_linear)
            {
                for (float i = (int) (min_x / grid_tick_v) * grid_tick_v; i < max_x; i += grid_tick_v)
                    ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_grid_v) + "] (" + std::to_string(i) +
                           "," + std::to_string(max_y) + ") -- (" + std::to_string(i) + "," + std::to_string(min_y) +
                           ");\n";
            } else if (axis_type_x == LaTeXTikz2D::axis_type_log10)
            {
                float decade, decade_old, tick;
                decade = decade_old = std::floor(min_x);
                for (tick = std::pow(10.0f, decade); std::log10(tick) < min_x; tick += pow(10.0f, decade) * grid_tick_v);
                decade = std::floor(std::log10(tick));
                if (decade != decade_old)
                {
                    tick = std::pow(10.0f, decade);
                    decade_old = decade;
                }

                while (std::log10(tick) < max_x)
                {
                    ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_grid_v) + "] (" +
                           std::to_string(std::log10(tick)) + "," + std::to_string(max_y) + ") -- (" +
                           std::to_string(std::log10(tick)) + "," + std::to_string(min_y) + ");\n";

                    tick += std::pow(10.0f, decade) * grid_tick_v;
                    decade = std::floor(std::log10(tick));
                    if (decade != decade_old)
                    {
                        tick = std::pow(10.0f, decade);
                        decade_old = decade;
                    }
                }
            }
        }
        ofs << "\n";
        if (has_grid_h)
        {
            if (axis_type_y == LaTeXTikz2D::axis_type_linear)
            {
                for (float i = (int) (min_y / grid_tick_h) * grid_tick_h; i < max_y; i += grid_tick_h)
                    ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_grid_h) + "] (" + std::to_string(max_x) +
                           "," + std::to_string(i) + ") -- (" + std::to_string(min_x) + "," + std::to_string(i) +
                           ");\n";
            } else if (axis_type_y == LaTeXTikz2D::axis_type_log10)
            {
                float decade, decade_old, tick;
                decade = decade_old = std::floor(min_y);
                for (tick = std::pow(10.0f, decade); std::log10(tick) < min_y; tick += std::pow(10.0f, decade) * grid_tick_h);
                decade = std::floor(std::log10(tick));
                if (decade != decade_old)
                {
                    tick = std::pow(10.0f, decade);
                    decade_old = decade;
                }

                while (std::log10(tick) < max_y)
                {
                    ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_grid_h) + "] (" + std::to_string(max_x) +
                           "," + std::to_string(std::log10(tick)) + ") -- (" + std::to_string(min_x) + "," +
                           std::to_string(std::log10(tick)) + ");\n";

                    tick += std::pow(10.0f, decade) * grid_tick_h;
                    decade = std::floor(std::log10(tick));
                    if (decade != decade_old)
                    {
                        tick = std::pow(10.0f, decade);
                        decade_old = decade;
                    }
                }
            }
        }
        ofs << "\n";

        // export axes
        if (has_axis_x)
        {
            if (axis_type_x == LaTeXTikz2D::axis_type_linear)
            {
                float i = (int) (min_x / label_tick_x) * label_tick_x;
                ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_x) + "] (" + std::to_string(min_x) +
                       "," + std::to_string(axis_x_position_v) + ") -- (" + std::to_string(i) + "," +
                       std::to_string(axis_x_position_v) + ");\n";
                ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_x) + "] (" + std::to_string(i) + "," +
                       std::to_string(axis_x_position_v) + "+" + LaTeXTikz2D::latex_var_mark_radius_y + ") -- (" +
                       std::to_string(i) + "," + std::to_string(axis_x_position_v) + "-" +
                       LaTeXTikz2D::latex_var_mark_radius_y + ");\n";
                ofs << "\t\\node[" + axis_num_position_x + "] at (" + std::to_string(i) + "," +
                       std::to_string(axis_x_position_v) + ") {" + numberToLatexString(i, axis_num_format_x) + "};\n";
                if (!axis_description_x.empty())
                {
                    ofs << "\t\\pgfgetlastxy{\\XCoord }{\\YCoord};\n";
                    ofs << "\t\\pgfmathsetmacro{\\DescXX }{\\XCoord}\n";
                    ofs << "\t\\pgfmathsetmacro{\\DescXY }{\\YCoord}\n";
                }
                for (i += label_tick_x; i < max_x; i += label_tick_x)
                {
                    ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_x) + "] (" +
                           std::to_string(i - label_tick_x) + "," + std::to_string(axis_x_position_v) + ") -- (" +
                           std::to_string(i) + "," + std::to_string(axis_x_position_v) + ");\n";
                    ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_x) + "] (" + std::to_string(i) +
                           "," + std::to_string(axis_x_position_v) + "+" + LaTeXTikz2D::latex_var_mark_radius_y +
                           ") -- (" + std::to_string(i) + "," + std::to_string(axis_x_position_v) + "-" +
                           LaTeXTikz2D::latex_var_mark_radius_y + ");\n";
                    ofs << "\t\\node[" + axis_num_position_x + "] at (" + std::to_string(i) + "," +
                           std::to_string(axis_x_position_v) + ") {" + numberToLatexString(i, axis_num_format_x) +
                           "};\n";
                    if (!axis_description_x.empty())
                    {
                        ofs << "\t\\pgfgetlastxy{\\XCoord }{\\YCoord};\n";
                        ofs << "\t\\pgfmathsetmacro{\\DescXY}{min(\\DescXY,\\YCoord)}\n";
                    }
                }
                if (!axis_description_x.empty())
                {
                    ofs << "\t\\pgfmathsetmacro{\\DescXX}{(\\DescXX+\\XCoord)/2};\n";
                }
                ofs << "\t\\draw[->,>=stealth," + std::string(LaTeXTikz2D::latex_style_axis_x) + "] (" +
                       std::to_string(i - label_tick_x) + "," + std::to_string(axis_x_position_v) + ") -- (" +
                       std::to_string(max_x) + "," + std::to_string(axis_x_position_v) + ");\n";
            }
            else if (axis_type_x == LaTeXTikz2D::axis_type_log10)
            {
                float decade, decade_old, tick, tick_old;
                decade = decade_old = std::floor(min_x);
                for (tick = std::pow(10.0f, decade); std::log10(tick) < min_x; tick += std::pow(10.0f, decade) * label_tick_x);
                tick_old = std::pow(10.0f, min_x);
                decade = std::floor(std::log10(tick));
                if (decade != decade_old)
                {
                    tick = std::pow(10.0f, decade);
                    decade_old = decade;
                }

                ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_x) + "] (" +
                       std::to_string(std::log10(tick_old)) + "," + std::to_string(axis_x_position_v) + ") -- (" +
                       std::to_string(std::log10(tick)) + "," + std::to_string(axis_x_position_v) + ");\n";
                ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_x) + "] (" +
                       std::to_string(std::log10(tick)) + "," + std::to_string(axis_x_position_v) + "+" +
                       LaTeXTikz2D::latex_var_mark_radius_y + ") -- (" + std::to_string(std::log10(tick)) + "," +
                       std::to_string(axis_x_position_v) + "-" + LaTeXTikz2D::latex_var_mark_radius_y + ");\n";
                ofs << "\t\\node[" + axis_num_position_x + "] at (" + std::to_string(std::log10(tick)) + "," +
                       std::to_string(axis_x_position_v) + ") {" + numberToLatexString(tick, axis_num_format_x) +
                       "};\n";
                if (!axis_description_x.empty())
                {
                    ofs << "\t\\pgfgetlastxy{\\XCoord }{\\YCoord};\n";
                    ofs << "\t\\pgfmathsetmacro{\\DescXX }{\\XCoord}\n";
                    ofs << "\t\\pgfmathsetmacro{\\DescXY }{\\YCoord}\n";
                }
                while (true)
                {
                    tick_old = tick;
                    tick += std::pow(10.0f, decade) * label_tick_x;
                    decade = std::floor(std::log10(tick));
                    if (decade != decade_old)
                    {
                        tick = std::pow(10.0f, decade);
                        decade_old = decade;
                    }
                    if (std::log10(tick) > max_x)
                        break;

                    ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_x) + "] (" +
                           std::to_string(std::log10(tick_old)) + "," + std::to_string(axis_x_position_v) + ") -- (" +
                           std::to_string(std::log10(tick)) + "," + std::to_string(axis_x_position_v) + ");\n";
                    ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_x) + "] (" +
                           std::to_string(std::log10(tick)) + "," + std::to_string(axis_x_position_v) + "+" +
                           LaTeXTikz2D::latex_var_mark_radius_y + ") -- (" + std::to_string(std::log10(tick)) + "," +
                           std::to_string(axis_x_position_v) + "-" + LaTeXTikz2D::latex_var_mark_radius_y + ");\n";
                    ofs << "\t\\node[" + axis_num_position_x + "] at (" + std::to_string(std::log10(tick)) + "," +
                           std::to_string(axis_x_position_v) + ") {" + numberToLatexString(tick, axis_num_format_x) +
                           "};\n";
                    if (!axis_description_x.empty())
                    {
                        ofs << "\t\\pgfgetlastxy{\\XCoord }{\\YCoord};\n";
                        ofs << "\t\\pgfmathsetmacro{\\DescXY}{min(\\DescXY,\\YCoord)}\n";
                    }
                }
                if (!axis_description_x.empty())
                {
                    ofs << "\t\\pgfmathsetmacro{\\DescXX}{(\\DescXX+\\XCoord)/2};\n";
                }
                ofs << "\t\\draw[->,>=stealth," + std::string(LaTeXTikz2D::latex_style_axis_x) + "] (" +
                       std::to_string(std::log10(tick_old)) + "," + std::to_string(axis_x_position_v) + ") -- (" +
                       std::to_string(max_x) + "," + std::to_string(axis_x_position_v) + ");\n";
            }
            if (!axis_description_x.empty())
            {
                ofs << "\t\\begin{scope}[reset cm]\n"
                       "\t\t\\coordinate (desc) at (\\DescXX pt, \\DescXY pt);\n"
                       "\t\\end{scope}\n"
                       "\t\\node[anchor=north,outer sep=5 pt] at(desc) {" + axis_description_x + "};\n";
            }
        }
        ofs << "\n";
        if (has_axis_y)
        {
            if (axis_type_y == LaTeXTikz2D::axis_type_linear)
            {
                float i = (int) (min_y / label_tick_y) * label_tick_y;
                ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_y) + "] (" +
                       std::to_string(axis_y_position_h) + "," + std::to_string(min_y) + ") -- (" +
                       std::to_string(axis_y_position_h) + "," + std::to_string(i) + ");\n";
                ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_y) + "] (" +
                       std::to_string(axis_y_position_h) + "+" + LaTeXTikz2D::latex_var_mark_radius_x + "," +
                       std::to_string(i) + ") -- (" + std::to_string(axis_y_position_h) + "-" +
                       LaTeXTikz2D::latex_var_mark_radius_x + "," + std::to_string(i) + ");\n";
                ofs << "\t\\node[" + axis_num_position_y + "] at (" + std::to_string(axis_y_position_h) + "," +
                       std::to_string(i) + ") {" + numberToLatexString(i, axis_num_format_y) + "};\n";
                if (!axis_description_y.empty())
                {
                    ofs << "\t\\pgfgetlastxy{\\XCoord }{\\YCoord};\n";
                    ofs << "\t\\pgfmathsetmacro{\\DescYX }{\\XCoord}\n";
                    ofs << "\t\\pgfmathsetmacro{\\DescYY }{\\YCoord}\n";
                }

                for (i += label_tick_y; i < max_y; i += label_tick_y)
                {
                    ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_y) + "] (" +
                           std::to_string(axis_y_position_h) + "," + std::to_string(i - label_tick_y) + ") -- (" +
                           std::to_string(axis_y_position_h) + "," + std::to_string(i) + ");\n";
                    ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_y) + "] (" +
                           std::to_string(axis_y_position_h) + "+" + LaTeXTikz2D::latex_var_mark_radius_x + "," +
                           std::to_string(i) + ") -- (" + std::to_string(axis_y_position_h) + "-" +
                           LaTeXTikz2D::latex_var_mark_radius_x + "," + std::to_string(i) + ");\n";
                    ofs << "\t\\node[" + axis_num_position_y + "] at (" + std::to_string(axis_y_position_h) + "," +
                           std::to_string(i) + ") {" + numberToLatexString(i, axis_num_format_y) + "};\n";
                    if (!axis_description_y.empty())
                    {
                        ofs << "\t\\pgfgetlastxy{\\XCoord }{\\YCoord};\n";
                        ofs << "\t\\pgfmathsetmacro{\\DescYX}{min(\\DescYX,\\XCoord)}\n";
                    }
                }
                if (!axis_description_y.empty())
                {
                    ofs << "\t\\pgfmathsetmacro{\\DescYY}{(\\DescYY+\\YCoord)/2};\n";
                }
                ofs << "\t\\draw[->,>=stealth," + std::string(LaTeXTikz2D::latex_style_axis_y) + "] (" +
                       std::to_string(axis_y_position_h) + "," + std::to_string(i - label_tick_y) + ") -- (" +
                       std::to_string(axis_y_position_h) + "," + std::to_string(max_y) + ");\n";
            }
            else if (axis_type_y == LaTeXTikz2D::axis_type_log10)
            {
                float decade, decade_old, tick, tick_old;
                decade = decade_old = std::floor(min_y);
                for (tick = std::pow(10.0f, decade); std::log10(tick) < min_y; tick += std::pow(10.0f, decade) * label_tick_y);
                tick_old = std::pow(10.0f, min_y);
                decade = std::floor(std::log10(tick));
                if (decade != decade_old)
                {
                    tick = std::pow(10.0f, decade);
                    decade_old = decade;
                }

                ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_y) + "] (" +
                       std::to_string(axis_y_position_h) + "," + std::to_string(std::log10(tick_old)) + ") -- (" +
                       std::to_string(axis_y_position_h) + "," + std::to_string(std::log10(tick)) + ");\n";
                ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_y) + "] (" +
                       std::to_string(axis_y_position_h) + "+" + LaTeXTikz2D::latex_var_mark_radius_x + "," +
                       std::to_string(std::log10(tick)) + ") -- (" + std::to_string(axis_y_position_h) + "-" +
                       LaTeXTikz2D::latex_var_mark_radius_x + "," + std::to_string(std::log10(tick)) + ");\n";
                ofs << "\t\\node[" + axis_num_position_y + "] at (" + std::to_string(axis_y_position_h) + "," +
                       std::to_string(std::log10(tick)) + ") {" + numberToLatexString(tick, axis_num_format_y) + "};\n";
                if (!axis_description_y.empty())
                {
                    ofs << "\t\\pgfgetlastxy{\\XCoord }{\\YCoord};\n";
                    ofs << "\t\\pgfmathsetmacro{\\DescYX }{\\XCoord}\n";
                    ofs << "\t\\pgfmathsetmacro{\\DescYY }{\\YCoord}\n";
                }

                while (true)
                {
                    tick_old = tick;
                    tick += std::pow(10.0f, decade) * label_tick_y;
                    decade = std::floor(std::log10(tick));
                    if (decade != decade_old)
                    {
                        tick = std::pow(10.0f, decade);
                        decade_old = decade;
                    }
                    if (std::log10(tick) > max_y)
                        break;

                    ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_y) + "] (" +
                           std::to_string(axis_y_position_h) + "," + std::to_string(std::log10(tick_old)) + ") -- (" +
                           std::to_string(axis_y_position_h) + "," + std::to_string(std::log10(tick)) + ");\n";
                    ofs << "\t\\draw[" + std::string(LaTeXTikz2D::latex_style_axis_y) + "] (" +
                           std::to_string(axis_y_position_h) + "+" + LaTeXTikz2D::latex_var_mark_radius_x + "," +
                           std::to_string(std::log10(tick)) + ") -- (" + std::to_string(axis_y_position_h) + "-" +
                           LaTeXTikz2D::latex_var_mark_radius_x + "," + std::to_string(std::log10(tick)) + ");\n";
                    ofs << "\t\\node[" + axis_num_position_y + "] at (" + std::to_string(axis_y_position_h) + "," +
                           std::to_string(std::log10(tick)) + ") {" + numberToLatexString(tick, axis_num_format_y) + "};\n";
                    if (!axis_description_y.empty())
                    {
                        ofs << "\t\\pgfgetlastxy{\\XCoord }{\\YCoord};\n";
                        ofs << "\t\\pgfmathsetmacro{\\DescYX}{min(\\DescYX,\\XCoord)}\n";
                    }
                }
                if (!axis_description_y.empty())
                {
                    ofs << "\t\\pgfmathsetmacro{\\DescYY}{(\\DescYY+\\YCoord)/2};\n";
                }
                ofs << "\t\\draw[->,>=stealth," + std::string(LaTeXTikz2D::latex_style_axis_y) + "] (" +
                       std::to_string(axis_y_position_h) + "," + std::to_string(std::log10(tick_old)) + ") -- (" +
                       std::to_string(axis_y_position_h) + "," + std::to_string(max_y) + ");\n";
            }
            if (!axis_description_y.empty())
            {
                ofs << "\t\\begin{scope}[reset cm]\n"
                       "\t\t\\coordinate (desc) at (\\DescYX pt, \\DescYY pt);\n"
                       "\t\\end{scope}\n"
                       "\t\\node[anchor=south,rotate=90,outer sep=5 pt] at(desc) {" + axis_description_y + "};\n";
            }
        }
        ofs << "\n";

        // export render code
        for (const auto & render_code : render_codes)
            ofs << render_code << "\n";

        // export finalization
        ofs << "\\end{tikzpicture}"
               "\n"
               "\\end{document}\n";

        ofs.close();
    }

    void LaTeXTikz2D::addGridH(const std::string &style, float tick)
    {
        latex_style_grid_h = saveStyle(style);
        grid_tick_h = tick;
        has_grid_h = true;
    }

    void LaTeXTikz2D::addGridV(const std::string &style, float tick)
    {
        latex_style_grid_v = saveStyle(style);
        grid_tick_v = tick;
        has_grid_v = true;
    }

    void LaTeXTikz2D::addAxisX(const std::string &style, std::string num_format, unsigned int num_position, float tick,
                               float crossing)
    {
        latex_style_axis_x = saveStyle(style);
        label_tick_x = tick;
        has_axis_x = true;
        axis_num_format_x = std::move(num_format);
        axis_num_position_x = positionToAnchor(num_position);
        axis_x_position_v = addY(crossing);
    }

    void LaTeXTikz2D::addAxisY(const std::string &style, std::string num_format, unsigned int num_position, float tick, float crossing)
    {
        latex_style_axis_y = saveStyle(style);
        label_tick_y = tick;
        has_axis_y = true;
        axis_num_format_y = std::move(num_format);
        axis_num_position_y = positionToAnchor(num_position);
        axis_y_position_h = addX(crossing);
    }
}