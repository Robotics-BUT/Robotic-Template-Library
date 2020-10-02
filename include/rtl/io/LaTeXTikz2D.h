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

#ifndef ROBOTICTEMPLATELIBRARY_IO_LATEXTIKZ2D_H
#define ROBOTICTEMPLATELIBRARY_IO_LATEXTIKZ2D_H

#include <string>
#include <utility>
#include <vector>
#include <map>
#include <limits>
#include <fstream>

#include "rtl/Core.h"
#include "rtl/Transformation.h"

namespace rtl
{
    //! LaTeX export of high quality vector graphics using the Tikz package - 2D drawing.
    /*!
     * This class is used to aggregate graphic primitives to be rendered into a PDF format. The rendering order corresponds to the order in which the primitives are added,
     * with an exception of axes and grids, which are always in the bottom. The graphics is gradually built using add* functions and exported to TEX file when it is finished.
     */
    class LaTeXTikz2D
    {
    public:
        //! Constructs the exporter with either linear or logarithmic scale on the axes.
        /*!
         * Axis type can be either LaTeXTikz2D::axis_type_linear, or LaTeXTikz2D::axis_type_log10.
         * @param axis_x \a x axis scale type, linear by default.
         * @param axis_y \a y axis scale type, linear by default.
         */
        explicit LaTeXTikz2D(unsigned int axis_x = LaTeXTikz2D::axis_type_linear, unsigned int axis_y = LaTeXTikz2D::axis_type_linear)
        {
            clearAll();
            axis_type_x = axis_x;
            axis_type_y = axis_y;
        }

        //! Default destructor.
        ~LaTeXTikz2D() = default;

        //! Sets size of the exported image in centimeters.
        /*!
         *
         * @param width width in centimeters.
         * @param height height in centimeters.
         */
        void setSize(float width, float height)
        {
            export_width = width;
            export_height = height;
        }

        //! Sets free space border around content of the picture.
        /*!
         *
         * @param border border width in centimeters.
         */
        void setBorder(float border)
        {
            export_border = border;
        }

        //! Sets mark radius.
        /*!
         *
         * @param radius mark radius in cetimeters.
         */
        void setMarkRadius(float radius)
        {
            mark_radius = radius;
        }

        //! Sets how much should axes and grids exceed the displayed content region.
        /*!
         *
         * @param type relative or absolute measure given by LaTeXTikz2D::overrun_relative and LaTeXTikz2D::overrun_absolute options.
         * @param magnitude overrun magnitude.
         */
        void setGridAxisOverrun(unsigned int type, float magnitude)
        {
            overrun_type = type;
            overrun_magnitude = magnitude;
        }

        //! Scales all coordinates in \a x axis direction.
        /*!
         *
         * @param scale scaling factor.
         */
        void setScaleX(float scale)
        {
            scale_x = scale;
        }

        //! Scales all coordinates in \a y axis direction.
        /*!
         *
         * @param scale scaling factor.
         */
        void setScaleY(float scale)
        {
            scale_y = scale;
        }

        //! Clipping of the rendered content.
        /*!
         * Clips region given by two points.
         * @param p1_x first point \a x coordinate.
         * @param p1_y first point \a y coordinate.
         * @param p2_x second point \a x coordinate.
         * @param p2_y second point \a y coordinate.
         */
        void setClipRegion(float p1_x, float p1_y, float p2_x, float p2_y)
        {
            is_clipped = true;
            clip_p1_x = p1_x;
            clip_p1_y = p1_y;
            clip_p2_x = p2_x;
            clip_p2_y = p2_y;
        }

        //! Sets minimal region to be plotted.
        /*!
         * Sets the region by two points.
         * @param p1_x first point \a x coordinate.
         * @param p1_y first point \a y coordinate.
         * @param p2_x second point \a x coordinate.
         * @param p2_y second point \a y coordinate.
         */
        void setMinPlotRegion(float p1_x, float p1_y, float p2_x, float p2_y)
        {
            addX(p1_x);
            addY(p1_y);
            addX(p2_x);
            addY(p2_y);
        }

        //! Clears all settings as well as data in the exporter.
        void clearAll()
        {
            clearData();

            scale_x = scale_y = 1.0f;
            mark_radius = 0.1f;
            export_width = export_height = 10.0f;
            export_border = 0.1f;
            clip_p1_x = clip_p1_y = clip_p2_x = clip_p2_y = 0.0f;
            is_clipped = false;
        }

        //! Clears only data, while export settings are left unchanged.
        void clearData()
        {
            styles.clear();
            marks.clear();
            colors.clear();
            render_codes.clear();

            max_x = max_y = -std::numeric_limits<float>::max();
            min_x = min_y = std::numeric_limits<float>::max();
            has_grid_h = has_grid_v = has_axis_x = has_axis_y = false;

            overrun_type = LaTeXTikz2D::overrun_relative;
            overrun_magnitude = 5.0f;
            label_tick_x = label_tick_y = grid_tick_h = grid_tick_v = 0.0f;
            axis_description_x = axis_description_y = axis_num_format_x = axis_num_format_y = axis_num_position_x = axis_num_position_y = "";
        }

        //! Writes internal data according to export settings into .tex file.
        /*!
         *
         * @param file_name output file path.
         */
        void writeTEX(const std::string &file_name)
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

        //! Adds horizontal lines to the plot.
        /*!
         *
         * @param style Tikz style.
         * @param tick interval between lines.
         */
        void addGridH(const std::string &style, float tick)
        {
            latex_style_grid_h = saveStyle(style);
            grid_tick_h = tick;
            has_grid_h = true;
        }

        //! Adds vertical lines to the plot.
        /*!
         *
         * @param style Tikz style.
         * @param tick interval between lines.
         */
        void addGridV(const std::string &style, float tick)
        {
            latex_style_grid_v = saveStyle(style);
            grid_tick_v = tick;
            has_grid_v = true;
        }

        //! Adds \a x axis to the plot.
        /*!
         *
         * @param style Tikz style.
         * @param num_format number display format as used in standard sprint() function.
         * @param num_position number position relative to the point it belongs to. Use one of LaTeXTikz2D::position_above, LaTeXTikz2D::position_below, LaTeXTikz2D::position_right, or LaTeXTikz2D::position_left.
         * @param tick interval between marks.
         * @param crossing point where the \a y axis should cross this one.
         */
        void addAxisX(const std::string &style, std::string num_format, unsigned int num_position, float tick, float crossing = 0.0f)
        {
            latex_style_axis_x = saveStyle(style);
            label_tick_x = tick;
            has_axis_x = true;
            axis_num_format_x = std::move(num_format);
            axis_num_position_x = positionToAnchor(num_position);
            axis_x_position_v = addY(crossing);
        }

        //! Adds \a y axis to the plot.
        /*!
         *
         * @param style Tikz style.
         * @param num_format number display format as used in standard sprint() function.
         * @param num_position number position relative to the point it belongs to. Use one of LaTeXTikz2D::position_above, LaTeXTikz2D::position_below, LaTeXTikz2D::position_right, or LaTeXTikz2D::position_left.
         * @param tick interval between marks.
         * @param crossing point where the \a x axis should cross this one.
         */
        void addAxisY(const std::string &style, std::string num_format, unsigned int num_position, float tick, float crossing = 0.0f)
        {
            latex_style_axis_y = saveStyle(style);
            label_tick_y = tick;
            has_axis_y = true;
            axis_num_format_y = std::move(num_format);
            axis_num_position_y = positionToAnchor(num_position);
            axis_y_position_h = addX(crossing);
        }

        //! Description for the \a x axis.
        /*!
         *
         * @param description LaTeX compliant description string.
         */
        void addDescriptionX(const std::string &description) { axis_description_x = description; }

        //! Description for the \a y axis.
        /*!
         *
         * @param description LaTeX compliant description string.
         */
        void addDescriptionY(const std::string &description) { axis_description_y = description; }

        //! Plots a set of points using given style.
        /*!
         *
         * @param x \a x coordinates of the points.
         * @param y \a y coordinates of the points.
         * @param line_style Tikz style for point connecting lines.
         * @param mark_style Tikz style for points themselves.
         * @param mark mark drawing code.
         * @param mark_scale individual scale of marks for given points.
         */
        void addPlot(const std::vector<float> &x, const std::vector<float> &y, const std::string &line_style, const std::string &mark_style = "", const std::string &mark = LaTeXTikz2D::latex_mark_blank, float mark_scale = 1.0f)
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

        //! Plots a set of points using given style.
        /*!
         *
         * @param v points to be rendered.
         * @param line_style Tikz style for point connecting lines.
         * @param mark_style Tikz style for points themselves.
         * @param mark mark drawing code.
         * @param mark_scale individual scale of marks for given points.
         */
        void addPlot(const std::vector<Vector2f> &v, const std::string &line_style, const std::string &mark_style = "", const std::string &mark = LaTeXTikz2D::latex_mark_blank, float mark_scale = 1.0f)
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

        //! Plots a set of line segments or other compliant objects.
        /*!
         *
         * @tparam T type of the objects to be rendered.
         * @param edges objects to be rendered.
         * @param style Tikz style.
         * @param options unused so far...
         */
        template<class T>
        void addEdges(const std::vector<T> &edges, const std::string &style, unsigned int options = 0);

        //! Plots a line segment or other compliant object.
        /*!
         *
         * @tparam T type of the object to be rendered.
         * @param edge object to be rendered.
         * @param style Tikz style.
         * @param options unused so far...
         */
        template<class T>
        void addEdge(const T &edge, const std::string &style, unsigned int options = 0);

        //! Plots a triangle using given style.
        /*!
         *
         * @param a point of th triangle.
         * @param b point of th triangle.
         * @param c point of th triangle.
         * @param style Tikz style.
         */
        void addTriangle(const Vector2f &a, const Vector2f &b, const Vector2f &c, const std::string &style)
        {
            std::string style_name = saveStyle(style);
            std::string code =
                    "\\filldraw[" + style_name + "] (" + std::to_string(addX(a.x())) + "," + std::to_string(addY(a.y())) +
                    ") -- (" + std::to_string(addX(b.x())) + "," + std::to_string(addY(b.y())) + ") -- (" +
                    std::to_string(addX(c.x())) + "," + std::to_string(addY(c.y())) + ") -- cycle;";
            render_codes.push_back(code);
        }

        //! Plots a rectangle using given style.
        /*!
         *
         * @param p1 one corner of the rectangle.
         * @param p2 diagonal corner of the rectangle.
         * @param style Tikz style.
         */
        void addRectangle(const Vector2f &p1, const Vector2f &p2, const std::string &style)
        {
            std::string style_name = saveStyle(style);
            std::string code = "\\filldraw[" + style_name + "] (" + std::to_string(addX(p1.x())) + ", " +
                               std::to_string(addY(p1.y())) + ") rectangle (" + std::to_string(addX(p2.x())) + ", " +
                               std::to_string(addY(p2.y())) + ");";
            render_codes.push_back(code);
        }

        //! Plots general quadrilateral using given style.
        /*!
         *
         * @param a point of the quadrilateral.
         * @param b point of the quadrilateral.
         * @param c point of the quadrilateral.
         * @param d point of the quadrilateral.
         * @param style Tikz style.
         */
        void addQuadrilateral(const Vector2f &a, const Vector2f &b, const Vector2f &c, const Vector2f &d, const std::string &style)
        {
            std::string style_name = saveStyle(style);
            std::string code =
                    "\\filldraw[" + style_name + "] (" + std::to_string(addX(a.x())) + "," + std::to_string(addY(a.y())) +
                    ") -- (" + std::to_string(addX(b.x())) + "," + std::to_string(addY(b.y())) + ") -- (" +
                    std::to_string(addX(c.x())) + "," + std::to_string(addY(c.y())) + ") -- (" +
                    std::to_string(addX(d.x())) + "," + std::to_string(addY(d.y())) + ") -- cycle;";
            render_codes.push_back(code);
        }

        //! Plots a circle using given style.
        /*!
         *
         * @param centre central point.
         * @param radius radius of the circle.
         * @param style Tikz style.
         */
        void addCircle(const Vector2f &centre, float radius, const std::string &style)
        {
            std::string style_name = saveStyle(style);
            std::string code = "\\filldraw[" + style_name + "] (" + std::to_string(addX(centre.x())) + ", " +
                               std::to_string(addY(centre.y())) + ") circle [radius=" + std::to_string(radius) + "];";
            render_codes.push_back(code);
        }

        //! Plots an ellipse using given style.
        /*!
         *
         * @param centre central point.
         * @param x_radius first semi-axis.
         * @param y_radius second semi-axis.
         * @param rotation rotation with respect to central point.
         * @param style Tikz style.
         */
        void addEllipse(const Vector2f &centre, float x_radius, float y_radius, float rotation, const std::string &style)
        {
            std::string style_name = saveStyle(style);
            std::string code = "\\filldraw[" + style_name + "] (" + std::to_string(addX(centre.x())) + ", " +
                               std::to_string(addY(centre.y())) + ") circle [x radius=" + std::to_string(x_radius) +
                               ", y radius=" + std::to_string(y_radius) + ", rotate=" +
                               std::to_string(rotation / rtl::C_PIf * 180.0f) + "];";
            render_codes.push_back(code);
        }

        //! Plots a pie using given style.
        /*!
         *
         * @param centre central point.
         * @param radius radius of the circle.
         * @param angle_beg angle of the pie beginning radius vector.
         * @param angle_end angle of the pie ending radius vector.
         * @param style Tikz style.
         */
        void addPie(const Vector2f &centre, float radius, float angle_beg, float angle_end, const std::string &style)
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

        //! Plots line given by two points using given style.
        /*!
         *
         * @param beg begin point.
         * @param end end point.
         * @param style Tikz style.
         */
        void addLine(const Vector2f &beg, const Vector2f &end, const std::string &style)
        {
            std::string style_name = saveStyle(style);
            std::string code =
                    "\\draw[" + style_name + "] (" + std::to_string(addX(beg.x())) + ", " + std::to_string(addY(beg.y())) +
                    ") -- (" + std::to_string(addX(end.x())) + ", " + std::to_string(addY(end.y())) + ");";
            render_codes.push_back(code);
        }

        //! Plots a 2D line segment using given style.
        /*!
         *
         * @param ls line segment to be plotted.
         * @param style Tikz style.
         */
        void addLine(const LineSegment2f &ls, const std::string &style)
        {
            addLine(ls.beg(), ls.end(), style);
        }

        //! Plots a std::vector of 2D line segments using given style.
        /*!
         *
         * @param ls Line segments to be plotted.
         * @param style Tikz style.
         */
        void addLines(const std::vector<LineSegment2f> &ls, const std::string &style)
        {
            for (auto &l : ls)
                addLine(l.beg(), l.end(), style);
        }

        //! Adds text to the plot.
        /*!
         *
         * @param text LaTeX compliant text string to be plotted.
         * @param style Tikz style of the text node.
         * @param position coordinates of the node.
         */
        void addText(const std::string &text, const std::string &style, const Vector2f &position)
        {
            std::string style_name = saveStyle(style);
            std::string code = "\\node[" + style_name + "] at (" + std::to_string(addX(position.x())) + "," +
                               std::to_string(addY(position.y())) + ") {" + text + "};";
            render_codes.push_back(code);
        }

        //! Returns maximal \a x coordinate to be plotted.
        /*!
         *
         * @return maximal \a x coordinate to be plotted.
         */
        float maxX()
        {
            return max_x;
        }

        //! Returns minimal \a x coordinate to be plotted.
        /*!
         *
         * @return minimal \a x coordinate to be plotted.
         */
        float minX()
        {
            return min_x;
        }

        //! Returns maximal \a y coordinate to be plotted.
        /*!
         *
         * @return maximal \a y coordinate to be plotted.
         */
        float maxY()
        {
            return max_y;
        }

        //! Returns minimal \a y coordinate to be plotted.
        /*!
         *
         * @return minimal \a y coordinate to be plotted.
         */
        float minY()
        {
            return min_y;
        }

        //! Returns current scaling of \a x coordinates.
        /*!
         *
         * @return \a x axis scaling.
         */
        float scaleX()
        {
            return scale_x;
        }

        //! Returns current scaling of \a y coordinates.
        /*!
         *
         * @return \a y axis scaling.
         */
        float scaleY()
        {
            return scale_y;
        }

        //! Sets overrun of the grids and axes to be relative to plot size. See setGridAxisOverrun().
        static const unsigned int overrun_relative = 0;
        //! Sets overrun of the grids and axes to be given by absolute value. See setGridAxisOverrun().
        static const unsigned int overrun_absolute = 1;

        //! Sets axis scaling to be linear.
        static const unsigned int axis_type_linear = 0;
        //! Sets axis scaling to be logarithmic.
        static const unsigned int axis_type_log10 = 1;

        //! Tikz relative positioning - above the node.
        static const unsigned int position_above = 0x01;
        //! Tikz relative positioning - below the node.
        static const unsigned int position_below = 0x02;
        //! Tikz relative positioning - right to the node.
        static const unsigned int position_right = 0x04;
        //! Tikz relative positioning - left to the node.
        static const unsigned int position_left = 0x08;

        //! Mark template - no mark at all, use this to produce plots with lines only.
        static inline const char *latex_mark_blank = "";
        //! Mark template - cross of two short lines.
        static inline const char *latex_mark_cross = "\\draw (-\\MarkRadius,-\\MarkRadius) -- (\\MarkRadius,\\MarkRadius);\n\t\\draw (\\MarkRadius,-\\MarkRadius) -- (-\\MarkRadius, \\MarkRadius);";
        //! Mark template - circular mark with outline and fill color.
        static inline const char *latex_mark_mark = "\\draw (0,-\\MarkRadius) -- (0,\\MarkRadius);";
        //! Mark template - one color dot.
        static inline const char *latex_mark_dot = "\\fill (0, 0) circle [radius=\\MarkRadius];\n\t\\draw (0, 0) circle [radius=\\MarkRadius];";
        //! Mark template - circular mark and heading line.
        static inline const char *latex_mark_robot = "\\fill (0, 0) circle [radius=\\MarkRadius];\n\t\\draw (0, 0) circle [radius=\\MarkRadius];\n\t\\draw (0, 0) -- (2*\\MarkRadius,0);";

        //! Converts number to LaTeX format using std::printf settings.
        /*!
         *
         * @tparam T number type.
         * @param num number value.
         * @param format string corresponding to std::printf standard.
         * @return string in LaTeX format.
         */
        template<typename T>
        static std::string numberToLatexString(T num, std::string format);

        //! Store given color.
        /*!
         *
         * @param color Tikz compliant color specification.
         * @return code for referencing the \p color in .tex files.
         */
        std::string saveColor(const std::string &color)
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

    private:
        std::map<std::string, std::string> styles;
        std::map<std::string, std::string> marks;
        std::map<std::string, std::string> colors;
        std::vector<std::string> render_codes;
        float scale_x{}, scale_y{}, max_x{}, min_x{}, max_y{}, min_y{};
        float mark_radius{}, export_width{}, export_height{}, export_border{};
        bool is_clipped{};
        float clip_p1_x{}, clip_p1_y{}, clip_p2_x{}, clip_p2_y{};

        // axes + grids
        bool has_grid_h{}, has_grid_v{}, has_axis_x{}, has_axis_y{};
        unsigned int overrun_type{}, axis_type_x, axis_type_y;
        float label_tick_x{}, label_tick_y{}, grid_tick_h{}, grid_tick_v{}, axis_x_position_v{}, axis_y_position_h{}, overrun_magnitude{};
        std::string axis_description_x, axis_description_y, axis_num_format_x, axis_num_format_y, axis_num_position_x, axis_num_position_y;

        static inline const char *latex_var_max_x = "\\MaxX";
        static inline const char *latex_var_min_x = "\\MinX";
        static inline const char *latex_var_max_y = "\\MaxY";
        static inline const char *latex_var_min_y = "\\MinY";
        static inline const char *latex_var_mark_radius_x = "\\MarkGlobalScaleX";
        static inline const char *latex_var_mark_radius_y = "\\MarkGlobalScaleY";

        std::string latex_style_grid_h;
        std::string latex_style_grid_v;
        std::string latex_style_axis_x;
        std::string latex_style_axis_y;

        float addX(float x)
        {
            if (axis_type_x == LaTeXTikz2D::axis_type_log10)
                x = std::log10(x);
            //x /= 10;
            if (x < min_x)
                min_x = x;
            if (x > max_x)
                max_x = x;
            return x;
        }

        float addY(float y)
        {
            if (axis_type_y == LaTeXTikz2D::axis_type_log10)
                y = std::log10(y);
            //y /= 10;
            if (y < min_y)
                min_y = y;
            if (y > max_y)
                max_y = y;
            return y;
        }

        std::string saveStyle(const std::string &style)
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

        std::string saveMark(const std::string &style_name, const std::string &mark_template, float scale)
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

        static std::string digitsToLetters(size_t num)
        {
            char buffer[50];
            int l = snprintf(buffer, sizeof(buffer), "%zd", num);
            if (l < 0)
                return std::string();
            for (int i = 0; i < l; i++)
                buffer[i] += 17;
            return std::string(buffer);
        }

        static std::string positionToAnchor(unsigned int pos)
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
    };


    template<typename T>
    std::string LaTeXTikz2D::numberToLatexString(T num, std::string format)
    {
        char buffer[50], f = format.back();
        switch (f)
        {
            case 'd':
            case 'i':
                snprintf(buffer, sizeof(buffer), format.c_str(), static_cast<long long int>(num));
                break;
            case 'u':
            case 'o':
            case 'x':
            case 'X':
                snprintf(buffer, sizeof(buffer), format.c_str(), static_cast<unsigned long long int>(num));
                break;
            case 'f':
            case 'F':
            case 'e':
            case 'E':
            case 'g':
            case 'G':
            case 'a':
            case 'A':
                snprintf(buffer, sizeof(buffer), format.c_str(), static_cast<double>(num));
                break;
            default:
                return std::string();
        }

        std::string str(buffer);
        std::size_t pos = str.find('e');
        if (pos == std::string::npos)
            pos = str.find('E');
        if (pos != std::string::npos)
        {
            if (str[pos + 1] == '+')
                str.erase(pos, 2);
            else
                str.erase(pos, 1);
            if (str[pos] == '-')
            {
                for (std::size_t i = pos + 1; i < str.size() - 1 && str[i] == '0'; i++)
                    str.erase(i, 1);
            } else
            {
                for (std::size_t i = pos; i < str.size() - 1 && str[i] == '0'; i++)
                    str.erase(i, 1);
            }
            if (pos == 1 && str[0] == '1')
            {
                str.erase(0, 1);
                str = "10^{" + str + '}';
            } else
            {
                str.insert(pos, " \\cdot 10^{");
                str.push_back('}');
            }
        }
        str.insert((std::size_t) 0, "$");
        str.push_back('$');
        return str;
    }


    template<class T>
    inline void LaTeXTikz2D::addEdges(const std::vector<T> &edges, const std::string &style, unsigned int)
    {
        std::string style_name = saveStyle(style);

        std::string code;
        for (size_t i = 0; i < edges.size(); i++)
            code += "\t\\draw[" + style_name + "] (" + std::to_string(addX(edges[i].beg().x())) + "," +
                    std::to_string(addY(edges[i].beg().y())) + ") -- (" + std::to_string(addX(edges[i].end().x())) +
                    "," + std::to_string(addY(edges[i].end().y())) + ");\n";

        render_codes.push_back(code);
    }

    template<class T>
    inline void LaTeXTikz2D::addEdge(const T &edge, const std::string &style, unsigned int)
    {
        std::string style_name = saveStyle(style);
        render_codes.push_back("\t\\draw[" + style_name + "] (" + std::to_string(addX(edge.beg().x())) + "," +
                               std::to_string(addY(edge.beg().y())) + ") -- (" + std::to_string(addX(edge.end().x())) +
                               "," + std::to_string(addY(edge.end().y())) + ");\n");
    }

}

#endif // ROBOTICTEMPLATELIBRARY_IO_LATEXTIKZ2D_H
