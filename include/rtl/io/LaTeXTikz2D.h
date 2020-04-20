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

#include "rtl/Core.h"

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
        explicit LaTeXTikz2D(unsigned int axis_x = LaTeXTikz2D::axis_type_linear, unsigned int axis_y = LaTeXTikz2D::axis_type_linear);

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
        void setBorder(float border) { export_border = border; }

        //! Sets mark radius.
        /*!
         *
         * @param radius mark radius in cetimeters.
         */
        void setMarkRadius(float radius) { mark_radius = radius; }

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
        void setScaleX(float scale) { scale_x = scale; }

        //! Scales all coordinates in \a y axis direction.
        /*!
         *
         * @param scale scaling factor.
         */
        void setScaleY(float scale) { scale_y = scale; }

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
        void clearAll();

        //! Clears only data, while export settings are left unchanged.
        void clearData();

        //! Writes internal data according to export settings into .tex file.
        /*!
         *
         * @param file_name output file path.
         */
        void writeTEX(const std::string &file_name);

        //! Adds horizontal lines to the plot.
        /*!
         *
         * @param style Tikz style.
         * @param tick interval between lines.
         */
        void addGridH(const std::string &style, float tick);

        //! Adds vertical lines to the plot.
        /*!
         *
         * @param style Tikz style.
         * @param tick interval between lines.
         */
        void addGridV(const std::string &style, float tick);

        //! Adds \a x axis to the plot.
        /*!
         *
         * @param style Tikz style.
         * @param num_format number display format as used in standard sprint() function.
         * @param num_position number position relative to the point it belongs to. Use one of LaTeXTikz2D::position_above, LaTeXTikz2D::position_below, LaTeXTikz2D::position_right, or LaTeXTikz2D::position_left.
         * @param tick interval between marks.
         * @param crossing point where the \a y axis should cross this one.
         */
        void addAxisX(const std::string &style, std::string num_format, unsigned int num_position, float tick, float crossing = 0.0f);

        //! Adds \a y axis to the plot.
        /*!
         *
         * @param style Tikz style.
         * @param num_format number display format as used in standard sprint() function.
         * @param num_position number position relative to the point it belongs to. Use one of LaTeXTikz2D::position_above, LaTeXTikz2D::position_below, LaTeXTikz2D::position_right, or LaTeXTikz2D::position_left.
         * @param tick interval between marks.
         * @param crossing point where the \a x axis should cross this one.
         */
        void addAxisY(const std::string &style, std::string num_format, unsigned int num_position, float tick, float crossing = 0.0f);

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
        void addPlot(const std::vector<float> &x, const std::vector<float> &y, const std::string &line_style, const std::string &mark_style = "", const std::string &mark = LaTeXTikz2D::latex_mark_blank, float mark_scale = 1.0f);

        //! Plots a set of points using given style.
        /*!
         *
         * @param v points to be rendered.
         * @param line_style Tikz style for point connecting lines.
         * @param mark_style Tikz style for points themselves.
         * @param mark mark drawing code.
         * @param mark_scale individual scale of marks for given points.
         */
        void addPlot(const std::vector<Vector2f> &v, const std::string &line_style, const std::string &mark_style = "", const std::string &mark = LaTeXTikz2D::latex_mark_blank, float mark_scale = 1.0f);

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
        void addTriangle(const Vector2f &a, const Vector2f &b, const Vector2f &c, const std::string &style);

        //! Plots a rectangle using given style.
        /*!
         *
         * @param p1 one corner of the rectangle.
         * @param p2 diagonal corner of the rectangle.
         * @param style Tikz style.
         */
        void addRectangle(const Vector2f &p1, const Vector2f &p2, const std::string &style);

        //! Plots general quadrilateral using given style.
        /*!
         *
         * @param a point of the quadrilateral.
         * @param b point of the quadrilateral.
         * @param c point of the quadrilateral.
         * @param d point of the quadrilateral.
         * @param style Tikz style.
         */
        void addQuadrilateral(const Vector2f &a, const Vector2f &b, const Vector2f &c, const Vector2f &d, const std::string &style);

        //! Plots a circle using given style.
        /*!
         *
         * @param centre central point.
         * @param radius radius of the circle.
         * @param style Tikz style.
         */
        void addCircle(const Vector2f &centre, float radius, const std::string &style);

        //! Plots an ellipse using given style.
        /*!
         *
         * @param centre central point.
         * @param x_radius first semi-axis.
         * @param y_radius second semi-axis.
         * @param rotation rotation with respect to central point.
         * @param style Tikz style.
         */
        void addEllipse(const Vector2f &centre, float x_radius, float y_radius, float rotation, const std::string &style);

        //! Plots a pie using given style.
        /*!
         *
         * @param centre central point.
         * @param radius radius of the circle.
         * @param angle_beg angle of the pie beginning radius vector.
         * @param angle_end angle of the pie ending radius vector.
         * @param style Tikz style.
         */
        void addPie(const Vector2f &centre, float radius, float angle_beg, float angle_end, const std::string &style);

        //! Plots line given by two points using given style.
        /*!
         *
         * @param beg begin point.
         * @param end end point.
         * @param style Tikz style.
         */
        void addLine(const Vector2f &beg, const Vector2f &end, const std::string &style);

        //! Plots a 2D line segment using given style.
        /*!
         *
         * @param ls line segment to be plotted.
         * @param style Tikz style.
         */
        void addLine(const LineSegment2f &ls, const std::string &style);

        //! Plots a std::vector of 2D line segments using given style.
        /*!
         *
         * @param ls Line segments to be plotted.
         * @param style Tikz style.
         */
        void addLines(const std::vector<LineSegment2f> &ls, const std::string &style);

        //! Adds text to the plot.
        /*!
         *
         * @param text LaTeX compliant text string to be plotted.
         * @param style Tikz style of the text node.
         * @param position coordinates of the node.
         */
        void addText(const std::string &text, const std::string &style, const Vector2f &position);

        //! Returns maximal \a x coordinate to be plotted.
        /*!
         *
         * @return maximal \a x coordinate to be plotted.
         */
        float maxX() { return max_x; }

        //! Returns minimal \a x coordinate to be plotted.
        /*!
         *
         * @return minimal \a x coordinate to be plotted.
         */
        float minX() { return min_x; }

        //! Returns maximal \a y coordinate to be plotted.
        /*!
         *
         * @return maximal \a y coordinate to be plotted.
         */
        float maxY() { return max_y; }

        //! Returns minimal \a y coordinate to be plotted.
        /*!
         *
         * @return minimal \a y coordinate to be plotted.
         */
        float minY() { return min_y; }

        //! Returns current scaling of \a x coordinates.
        /*!
         *
         * @return \a x axis scaling.
         */
        float scaleX() { return scale_x; }

        //! Returns current scaling of \a y coordinates.
        /*!
         *
         * @return \a y axis scaling.
         */
        float scaleY() { return scale_y; }

        //! Sets overrun of the grids and axes to be relative to plot size. See setGridAxisOverrun().
        static const unsigned int overrun_relative;
        //! Sets overrun of the grids and axes to be given by absolute value. See setGridAxisOverrun().
        static const unsigned int overrun_absolute;

        //! Sets axis scaling to be linear.
        static const unsigned int axis_type_linear;
        //! Sets axis scaling to be logarithmic.
        static const unsigned int axis_type_log10;

        //! Tikz relative positioning - above the node.
        static const unsigned int position_above;
        //! Tikz relative positioning - below the node.
        static const unsigned int position_below;
        //! Tikz relative positioning - right to the node.
        static const unsigned int position_right;
        //! Tikz relative positioning - left to the node.
        static const unsigned int position_left;

        //! Mark template - no mark at all, use this to produce plots with lines only.
        static const char *latex_mark_blank;
        //! Mark template - cross of two short lines.
        static const char *latex_mark_cross;
        //! Mark template - circular mark with outline and fill color.
        static const char *latex_mark_mark;
        //! Mark template - one color dot.
        static const char *latex_mark_dot;
        //! Mark template - circular mark and heading line.
        static const char *latex_mark_robot;

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
        std::string saveColor(const std::string &color);

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

        static const char *latex_var_max_x;
        static const char *latex_var_min_x;
        static const char *latex_var_max_y;
        static const char *latex_var_min_y;
        static const char *latex_var_mark_radius_x;
        static const char *latex_var_mark_radius_y;

        std::string latex_style_grid_h;
        std::string latex_style_grid_v;
        std::string latex_style_axis_x;
        std::string latex_style_axis_y;

        float addX(float x);

        float addY(float y);

        std::string saveStyle(const std::string &style);

        std::string saveMark(const std::string &style_name, const std::string &mark_template, float scale);

        static std::string digitsToLetters(size_t num);

        static std::string positionToAnchor(unsigned int pos);
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
