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

#ifndef ROBOTICTEMPLATELIBRARY_IO_LATEXTIKZ3D_H
#define ROBOTICTEMPLATELIBRARY_IO_LATEXTIKZ3D_H

#include <string>
#include <map>
#include <list>
#include <cmath>
#include <memory>
#include <utility>
#include <fstream>

#include "rtl/Core.h"
#include "rtl/Transformation.h"

namespace rtl
{
    //! LaTeX export of high quality vector graphics using the Tikz package - 3D scene rendering into 2D drawing.
    /*!
     * This class is used to aggregate graphic primitives to be rendered into a PDF format. The rendering order is determined using traditional visibility testing and do not require
     * any input from the user side. View of the scene can be set using regular translation-rotation-projection scheme.
     *
     * More complicated intersections of polygonal faces may result in visual artifacts appearing as darker lines. This happens, when two polygons of the same color are next to
     * each other and is a rendering bug in many .pdf viewers caused by an antialiasing algorithm. If perfect results are desired, the vector graphics can be converted to raster
     * with a suitable software (e.g. Gimp), or the generated code can be manually edited to merge the neighbouring polygons of the same style.
     */
    class LaTeXTikz3D
    {
    public:
        //! Default constructor with basic initialization of the exporter.
        LaTeXTikz3D()
        {
            clearAll();
        }

        //! Move constructor.
        LaTeXTikz3D(LaTeXTikz3D &&le) noexcept
        {
            styles.swap(le.styles);
            marks.swap(le.marks);
            colors.swap(le.colors);
            adapting_objects.swap(le.adapting_objects);
            fixed_objects.swap(le.fixed_objects);

            mark_primitives.swap(le.mark_primitives);
            line_primitives.swap(le.line_primitives);
            polygon_primitives.swap(le.polygon_primitives);
            render_primitives.swap(render_primitives);

            epsilon = le.epsilon;
            export_width = le.export_width;
            export_height = le.export_height;
            export_border = le.export_border;
            focal_length = le.focal_length;

            view_orientation = le.view_orientation;
            min_reg = std::move(le.min_reg);
            max_reg = std::move(le.max_reg);
            render_reg = std::move(le.render_reg);
            clipping = std::move(le.clipping);

            overrun_type = le.overrun_type;
            overrun_magnitude = le.overrun_magnitude;

        }

        //! Default destructor.
        ~LaTeXTikz3D() = default;

        //! Sets size of the exported image in centimeters.
        /*!
         *
         * @param width width in centimeters.
         * @param height height in centimeters.
         */
        void setExportSize(float width, float height)
        {
            export_width = width;
            export_height = height;
        }

        //! Clipping of the rendered content.
        /*!
         * Clips region given by two points.
         * @param p1_x first point \a x coordinate.
         * @param p1_y first point \a y coordinate.
         * @param p2_x second point \a x coordinate.
         * @param p2_y second point \a y coordinate.
         */
        void setExportClipping(float p1_x, float p1_y, float p2_x, float p2_y)
        {
            clipping = std::make_unique<BoundingBox2f>(Vector2f(p1_x, p1_y), Vector2f(p2_x, p2_y));
        }

        //! Sets transformation of the whole scene and field of view of the camera.
        /*!
         *
         * @param fov field of view in degrees.
         * @param orientation transformation applied on the scene before rendering.
         */
        void setView(float fov, const RigidTf3f &orientation)
        {
            focal_length = 1.0f / std::tan(fov / 180.0f * rtl::C_PIf / 2.0f);
            view_orientation = orientation;
        }

        //! Sets a direction from which the scene will be observed and a field of view of the camera.
        /*!
         * Direction of observation is given by \p camera_dir and scaling and translation is computed to fit the scene into the view as well as possible.
         * @param fov field view in degrees.
         * @param camera_dir direction vector from which the scene is observed.
         */
        void setView(float fov, const Vector3f &camera_dir)
        {
            focal_length = 1.0f / std::tan(fov / 180.0f * rtl::C_PIf / 2.0f);
            view_orientation = RigidTf3f(Quaternionf(-Vector3f::baseZ(), camera_dir), Vector3f::nan());
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

        //! Enables and sets style of a frame around the exported figure.
        /*!
         *
         * @param style Tikz style of the frame.
         */
        void setFrameStyle(const std::string &style)
        {
            frame_style = style;
        }

        //! Sets how much should axes and grids exceed the displayed content region.
        /*!
         *
         * @param type relative or absolute measure given by LaTeXTikz3D::overrun_relative and LaTeXTikz3D::overrun_absolute options.
         * @param magnitude overrun magnitude.
         */
        void setGridAxisOverrun(unsigned int type, float magnitude)
        {
            overrun_type = type;
            overrun_magnitude = magnitude;
        }

        //! Sets maximal bounding box to be plotted.
        /*!
         *
         * @param p1 first vertex of the bounding box.
         * @param p2 vertex diagonal to the first one.
         */
        void setMaxPlotRegion(const Vector3f &p1, const Vector3f &p2)
        {
            max_reg = std::make_unique<BoundingBox3f>(p1, p2);
        }

        //! Sets minimal bounding box to be plotted.
        /*!
         *
         * @param p1 first vertex of the bounding box.
         * @param p2 vertex diagonal to the first one.
         */
        void setMinPlotRegion(const Vector3f &p1, const Vector3f &p2)
        {
            min_reg = std::make_unique<BoundingBox3f>(p1, p2);
        }

        //! Clears all settings as well as data in the exporter.
        void clearAll()
        {
            clearData();
            export_width = export_height = 10.0f;
            export_border = 0.1f;
            min_reg = nullptr;
            max_reg = nullptr;
            frame_style.clear();
        }

        //! Clears only data, while export settings are left unchanged.
        void clearData()
        {
            styles.clear();
            marks.clear();
            colors.clear();
            adapting_objects.clear();
            fixed_objects.clear();
            render_primitives.clear();

            overrun_type = LaTeXTikz3D::overrun_relative;
            overrun_magnitude = 5.0f;
        }

        //! Writes internal data according to export settings into .tex file.
        /*!
         *
         * @param file_name output file path.
         */
        void writeTEX(const std::string &file_name)
        {
            if (!min_reg && fixed_objects.empty())
                return;

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
                                                                                     "\\begin{tikzpicture}[\n";

            // export styles
            if (!styles.empty())
            {
                auto it_last = --styles.end();
                for (auto it = styles.begin(); it != it_last; it++)
                    ofs << "\t" << it->second << "/." << it->first << ",\n";
                ofs << "\t" << it_last->second << "/." << it_last->first << "\n";
            }
            ofs << "\t]\n\n";

            // export clip region
            ofs << "\t\\clip (" + std::to_string(export_width / 2) + "," + std::to_string(export_height / 2) + ") rectangle (" +
                   std::to_string(-export_width / 2) + "," + std::to_string(-export_height / 2) + ");\n\n";
            if(!frame_style.empty())
                ofs << "\t\\draw[" + frame_style + "] (" + std::to_string(export_width / 2) + "," + std::to_string(export_height / 2) + ") rectangle (" +
                       std::to_string(-export_width / 2) + "," + std::to_string(-export_height / 2) + ");\n\n";

            // export colors
            for (auto &color : colors)
                ofs << "\t" << "\\definecolor{" + color.second + "}" + color.first << "\n";
            ofs << "\n";

            // export marks
            for (auto &mark : marks)
                ofs << "\t" << "\\newcommand{\\" + mark.second + "}[4]{" << mark.first << "\n\t}\n";
            ofs << "\n";

            // compute total bounding box
            if (min_reg)
            {
                render_reg = std::make_unique<BoundingBox3f>(*min_reg);
                for (const auto &fo : fixed_objects)
                    render_reg->addBoundingBox(fo->boundingBox());
            }
            else
            {
                auto fo_it = fixed_objects.begin();
                render_reg = std::make_unique<BoundingBox3f>((*fo_it)->boundingBox());
                for (fo_it++;fo_it != fixed_objects.end(); fo_it++)
                    render_reg->addBoundingBox((*fo_it)->boundingBox());
            }
            if (max_reg)
                render_reg = BoundingBox3f::intersection(*render_reg, *max_reg);

            // if semi-automatic view orientation is used, compute it now
            if (view_orientation.trVec().hasNaN())
            {
                auto render_reg_centre = view_orientation.rotMat() * render_reg->centroid();
                view_orientation.setTrVec({-render_reg_centre.x(), -render_reg_centre.y(), 0});
                float aspect_ratio = export_height / export_width;

                auto z_shift_required = [this, &aspect_ratio](const Vector3f &v)
                {
                    auto v_proj = view_orientation(v);
                    Vector2f z_to_edge;
                    z_to_edge.setX(-focal_length * std::abs(v_proj.x()) - v_proj.z());
                    z_to_edge.setY(-focal_length * std::abs(v_proj.y()) / aspect_ratio - v_proj.z());
                    return z_to_edge;
                };

                BoundingBox2f z_shift_bounds(render_reg->allVertices(z_shift_required));
                float z_shift = std::min(z_shift_bounds.min().x(), z_shift_bounds.min().y());
                view_orientation.setTrVec({-render_reg_centre.x(), -render_reg_centre.y(), z_shift});
            }

            // resize adapting objects
            for (auto &ao : adapting_objects)
                ao->fitTo(*render_reg);

            // extract render primitives
            render_primitives.clear();
            for (auto &fo : fixed_objects)
                fo->primitives(mark_primitives, line_primitives, polygon_primitives);
            for (auto &ao : adapting_objects)
                ao->primitives(mark_primitives, line_primitives, polygon_primitives);

            // project render primitives
            for (auto &mp : mark_primitives)
                mp->project(view_orientation, focal_length);
            for (auto &lp : line_primitives)
                lp->project(view_orientation, focal_length);
            for (auto &pp : polygon_primitives)
                pp->project(view_orientation, focal_length);

            // split and sort polygons
            struct BSPNode
            {
                std::list<std::unique_ptr<PolygonPrimitive>> primitives;
                std::unique_ptr<BSPNode> under{nullptr}, above{nullptr};
            };

            auto lambda_polygon_split_and_sort = [this](std::unique_ptr<BSPNode> &node) -> void
            {
                // added second lambda for recursive calls with auto type, see: http://pedromelendez.com/blog/2015/07/16/recursive-lambdas-in-c14/
                auto lambda_self_ref = [this](std::unique_ptr<BSPNode> &node, const auto &lambda_self_ref) -> void
                {
                    node->above = std::make_unique<BSPNode>();
                    node->under = std::make_unique<BSPNode>();
                    auto it = std::next(node->primitives.begin());
                    while (it != node->primitives.end())
                    {
                        node->primitives.front().get()->splitSort(std::move(*it), node->under->primitives, node->above->primitives, focal_length, epsilon);
                        it = node->primitives.erase(it);
                    }

                    if (node->above->primitives.empty())
                        node->above.reset(nullptr);
                    else if (node->above->primitives.size() > 1)
                        lambda_self_ref(node->above, lambda_self_ref);

                    if (node->under->primitives.empty())
                        node->under.reset(nullptr);
                    else if (node->under->primitives.size() > 1)
                        lambda_self_ref(node->under, lambda_self_ref);
                };
                lambda_self_ref(node, lambda_self_ref);
            };

            std::unique_ptr<BSPNode> root(new BSPNode);
            root->primitives.swap(polygon_primitives);
            if (!root->primitives.empty())
                lambda_polygon_split_and_sort(root);

            auto lambda_render = [this](std::unique_ptr<BSPNode> &node) -> void
            {
                // added second lambda for recursive calls with auto type, see: http://pedromelendez.com/blog/2015/07/16/recursive-lambdas-in-c14/
                auto lambda_self_ref = [this](std::unique_ptr<BSPNode> &node, const auto &lambda_self_ref) -> void
                {
                    if (node->primitives.front()->frontVisible())
                    {
                        if (node->under)
                            lambda_self_ref(node->under, lambda_self_ref);
                        render_primitives.push_back(std::move(node->primitives.front()));
                        if (node->above)
                            lambda_self_ref(node->above, lambda_self_ref);
                    }
                    else
                    {
                        if (node->above)
                            lambda_self_ref(node->above, lambda_self_ref);
                        render_primitives.push_back(std::move(node->primitives.front()));
                        if (node->under)
                            lambda_self_ref(node->under, lambda_self_ref);
                    }
                };
                lambda_self_ref(node, lambda_self_ref);
            };

            if (!root->primitives.empty())
                lambda_render(root);

            // sort lines - temporary, buggy, FIX NEEDED!!!
            auto lambda_line_order = [this](std::unique_ptr<LinePrimitive> &rp1, std::unique_ptr<LinePrimitive> &rp2)
            {
                std::unique_ptr<LinePrimitive> above, under;
                rp1->splitSort(std::move(rp2), under, above, focal_length, epsilon);
                if (under)
                {
                    rp1.swap(under);
                    rp2.swap(under);
                    return true;
                }
                else
                {
                    rp2.swap(above);
                    return false;
                }
            };
            line_primitives.sort(lambda_line_order);

            // merge lines
            std::unique_ptr<LinePrimitive> labove, lunder;
            while (!line_primitives.empty())
            {
                auto lp = std::move(line_primitives.back());
                line_primitives.pop_back();

                auto rp_rit = render_primitives.rbegin();
                while (rp_rit != render_primitives.rend())
                {
                    rp_rit->get()->splitSort(std::move(lp), lunder, labove, focal_length, epsilon);
                    if (!rp_rit->get()->frontVisible())
                        labove.swap(lunder);
                    if (labove)
                        render_primitives.insert(rp_rit.base(), std::move(labove));
                    if (!lunder)
                        goto new_line;
                    else
                    {
                        lp = std::move(lunder);
                        rp_rit++;
                    }
                }
                render_primitives.push_front(std::move(lp));
                new_line:;
            }

            // sort marks
            mark_primitives.sort([](const std::unique_ptr<MarkPrimitive> &rp1, const std::unique_ptr<MarkPrimitive> &rp2){ return rp1->pos_3d.lengthSquared() > rp2->pos_3d.lengthSquared();});

            // merge marks into render primitives
            std::unique_ptr<MarkPrimitive> above, under;
            while (!mark_primitives.empty())
            {
                auto mp = std::move(mark_primitives.back());
                mark_primitives.pop_back();

                auto rp_rit = render_primitives.rbegin();
                while (rp_rit != render_primitives.rend())
                {
                    rp_rit->get()->splitSort(std::move(mp), under, above, focal_length, epsilon);
                    if (!rp_rit->get()->frontVisible())
                        above.swap(under);
                    if (above)
                    {
                        render_primitives.insert(rp_rit.base(), std::move(above));
                        goto new_point;
                    }
                    else
                    {
                        mp = std::move(under);
                        rp_rit++;
                    }
                }
                render_primitives.push_front(std::move(mp));
                new_point:;
            }

            // render code
            float scale = std::max(export_width, export_height) / 2.0f;
            for (const auto &rp : render_primitives)
                ofs << rp->render(scale);

            // export finalization
            ofs << "\\end{tikzpicture}"
                   "\n"
                   "\\end{document}\n";

            ofs.close();
        }

        //! Adds an axis to the rendering.
        /*!
         * Adds arbitrary axis with numbering - the axis does not need to be aligned with anything. Axes are adapting objects and are scaled with respect to the whole scene.
         * @param style Tikz style of the axis.
         * @param num_format numbering format in std::printf-like manner.
         * @param num_position relative number position. Use one of LaTeXTikz3D::position_above, LaTeXTikz3D::position_below, LaTeXTikz3D::position_right, or LaTeXTikz3D::position_left.
         * @param tick period of numbering.
         * @param beg begin point of the axis.
         * @param end end point of the axis.
         */
        void addAxis(const std::string &style, std::string num_format, unsigned int num_position, float tick, const Vector3f &beg, const Vector3f &end)
        {
            adapting_objects.push_back(std::make_unique<Axis>(saveStyle(style), std::move(num_format), num_position, tick, beg, end));
        }

        //! Adds a mark to the rendering.
        /*!
         *
         * @param pos position of the mark in space.
         * @param mark_style Tikz style.
         * @param mark_template Tikz drawing code of the mark.
         * @param rotation rotation of he mark in the projection plane.
         * @param mark_radius individual scaling of the mark.
         */
        void addMark(const Vector3f &pos, const std::string &mark_style, const std::string &mark_template, float rotation = 0.0f, float mark_radius = 1.0f)
        {
            fixed_objects.push_back(std::make_unique<Mark>(pos, saveMark(saveStyle(mark_style), mark_template, mark_radius), rotation, mark_radius));
        }

        //! Adds multiple marks to the rendering.
        /*!
         *
         * @param vpos positions of the marks.
         * @param mark_style Tikz style.
         * @param mark_template Tikz drawing code of the mark.
         * @param rotation rotation of he mark in the projection plane.
         * @param mark_radius individual scaling of the mark.
         */
        void addMarks(const std::vector<Vector3f> &vpos, const std::string &mark_style, const std::string &mark_template, float rotation = 0.0f, float mark_radius = 1.0f)
        {
            std::string mark_key = saveMark(saveStyle(mark_style), mark_template, mark_radius);
            for (const Vector3f &v : vpos)
                fixed_objects.push_back(std::make_unique<Mark>(v, mark_key, rotation, mark_radius));
        }

        //! Adds a line to the rendering.
        /*!
         *
         * @param ls line segment.
         * @param style Tikz style.
         */
        void addLine(const LineSegment3f &ls, const std::string &style)
        {
            fixed_objects.push_back(std::make_unique<Edge>(saveStyle(style), ls));
        }

        //! Adds multiple lines to the rendering.
        /*!
         *
         * @param vls line segments to be rendered.
         * @param style Tikz style.
         */
        void addLines(const std::vector<LineSegment3f> &vls, const std::string &style)
        {
            std::string style_key = saveStyle(style);
            for (const LineSegment3f &ls : vls)
                fixed_objects.push_back(std::make_unique<Edge>(style_key, ls));
        }

        //! Adds a polygonal face to the rendering.
        /*!
         *
         * @param face polygonal face.
         * @param front_style Tikz style of the front side (the side from which the normal vector origins).
         * @param back_style Tikz style of the front side.
         * @param line_style Tikz style of the outline.
         */
        void addFace(const Polygon3Df &face, const std::string &front_style, const std::string &back_style, const std::string &line_style)
        {
            if (face.points().size() > 2)
                fixed_objects.push_back(std::make_unique<Face>(face, saveStyle(front_style), saveStyle(back_style), saveStyle(line_style)));
        }

        //! Adds multiple polygonal faces to the rendering.
        /*!
         *
         * @param vfs polygons to be rendered.
         * @param front_style Tikz style of the front side (the side from which the normal vector origins).
         * @param back_style Tikz style of the front side.
         * @param line_style Tikz style of the outline.
         */
        void addFace(const std::vector<Polygon3Df> &vfs, const std::string &front_style, const std::string &back_style, const std::string &line_style)
        {
            auto front_style_key = saveStyle(front_style);
            auto back_style_key = saveStyle(back_style);
            auto line_style_key = saveStyle(line_style);

            for(const auto &f : vfs)
                fixed_objects.push_back(std::make_unique<Face>(f, front_style_key, back_style_key, line_style_key));
        }

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

    private:
        struct MarkPrimitive;
        struct LinePrimitive;
        struct PolygonPrimitive;

        struct RenderPrimitive
        {
            virtual ~RenderPrimitive() = default;

            virtual void project(const RigidTf3f &tr, float fl) = 0;
            virtual std::string render(float scale) = 0;
            virtual RenderPrimitive* ptr() = 0;
            virtual bool frontVisible() = 0;

            virtual void splitSort(std::unique_ptr<MarkPrimitive>, std::unique_ptr<MarkPrimitive> &, std::unique_ptr<MarkPrimitive> &, float, float) {}
            virtual void splitSort(std::unique_ptr<LinePrimitive>, std::unique_ptr<LinePrimitive> &, std::unique_ptr<LinePrimitive> &, float, float) {}
            virtual void splitSort(std::unique_ptr<PolygonPrimitive>, std::list<std::unique_ptr<PolygonPrimitive>> &, std::list<std::unique_ptr<PolygonPrimitive>> &, float, float) {}

        };

        struct MarkPrimitive : RenderPrimitive
        {
            MarkPrimitive() = default;
            MarkPrimitive(const Vector3f &pos, std::string mn, float rot, float rad) : pos_3d{pos}, rotation{rot}, radius{rad}, mark{std::move(mn)} {}
            ~MarkPrimitive() override = default;

            void project(const RigidTf3f &tr, float fl) override
            {
                pos_3d = tr(pos_3d);
                scale = -fl / pos_3d.z();
                proj_2d = Vector2f(pos_3d.x(), pos_3d.y()) * scale;
            }

            std::string render(float sc) override
            {
                return "\t\\" + mark + "{" + std::to_string(proj_2d.x() * sc) + "}{" + std::to_string(proj_2d.y() * sc) + "}{" + std::to_string(rotation) + "}{" + std::to_string(scale * sc) + "}\n";
            }

            void splitSort(std::unique_ptr<MarkPrimitive> mp, std::unique_ptr<MarkPrimitive> &under, std::unique_ptr<MarkPrimitive> &above, [[maybe_unused]]float fl, [[maybe_unused]]float eps) override
            {
                if (pos_3d.lengthSquared() > mp->pos_3d.lengthSquared())
                    above = std::move(mp);
                else
                    under = std::move(mp);
            }

            MarkPrimitive* ptr() override { return this; }
            bool frontVisible() override { return true; }

            Vector3f pos_3d;
            Vector2f proj_2d;
            float rotation{}, scale{}, radius{};
            std::string mark;
        };

        struct LinePrimitive : RenderPrimitive
        {
            LinePrimitive() = default;
            LinePrimitive(const LineSegment3f &ls, std::string st) : ls_3d{ls}, style{std::move(st)} {}
            ~LinePrimitive() override = default;

            void project(const RigidTf3f &tr, float fl) override
            {
                ls_3d = tr(ls_3d);
                proj_2d.setBegin(-fl * ls_3d.beg().x() / ls_3d.beg().z(), -fl * ls_3d.beg().y() / ls_3d.beg().z());
                proj_2d.setEnd(-fl * ls_3d.end().x() / ls_3d.end().z(), -fl * ls_3d.end().y() / ls_3d.end().z());
            }

            std::string render(float scale) override
            {
                return "\t\\draw[" + style + "] (" + std::to_string(proj_2d.beg().x() * scale) + "," + std::to_string(proj_2d.beg().y() * scale) +
                       ") -- (" + std::to_string(proj_2d.end().x() * scale) + "," + std::to_string(proj_2d.end().y() * scale) + ");\n";
            }

            LinePrimitive* ptr() override { return this; }
            bool frontVisible() override { return true; }

            void splitSort(std::unique_ptr<MarkPrimitive> mp, std::unique_ptr<MarkPrimitive> &under, std::unique_ptr<MarkPrimitive> &above, float fl, float eps) override
            {
                float d = ls_3d.direction().cross(mp->pos_3d - ls_3d.beg()).lengthSquared();
                if (d < eps)
                {
                    above = std::move(mp);
                    return;
                }

                if (proj_2d.distanceToPointSquared(mp->proj_2d) < std::pow(mp->radius * mp->scale, 2))
                {
                    auto t_z = (mp->pos_3d.z() - ls_3d.beg().z()) / (ls_3d.end().z() - ls_3d.beg().z());
                    auto m_plane_intersection = -Vector2f(ls_3d.end().x() - ls_3d.beg().x(), ls_3d.end().y() - ls_3d.beg().y()) * t_z * fl / mp->pos_3d.z();
                    auto i_on_l = proj_2d.scalarProjection(m_plane_intersection);
                    auto m_on_l = proj_2d.scalarProjection(mp->proj_2d);
                    if ((ls_3d.direction().z() > 0) ^ (i_on_l > m_on_l))
                        under = std::move(mp);
                    else
                        above = std::move(mp);
                }
                else
                    under = std::move(mp); // "does not matter" is correct in principle, just used "under" in bipolar decision scheme
            }

            void splitSort(std::unique_ptr<LinePrimitive> lp, std::unique_ptr<LinePrimitive> &under, std::unique_ptr<LinePrimitive> &above, [[maybe_unused]]float fl, float eps) override
            {
                float t1, t2;
                LineSegment2f::getCrossing(proj_2d, lp->proj_2d, t1, t2);
                if (t1 > eps && t1 < 1.0 - eps && t2 > eps && t2 < 1 - eps)
                {
                    Vector2f crossing(proj_2d.beg() + t1 * (proj_2d.end() - proj_2d.beg()));
                    float l1, l2;
                    t1 = crossing.x() * ls_3d.direction().z() - ls_3d.direction().x();
                    t2 = crossing.y() * ls_3d.direction().z() - ls_3d.direction().y();
                    if (std::abs(t1) > std::abs(t2))
                        l1 = (ls_3d.beg() + ls_3d.direction() * (ls_3d.beg().x() - crossing.x() * ls_3d.beg().z()) / t1).lengthSquared();
                    else
                        l1 = (ls_3d.beg() + ls_3d.direction() * (ls_3d.beg().y() - crossing.y() * ls_3d.beg().z()) / t2).lengthSquared();

                    t1 = crossing.x() * lp->ls_3d.direction().z() - lp->ls_3d.direction().x();
                    t2 = crossing.y() * lp->ls_3d.direction().z() - lp->ls_3d.direction().y();
                    if (std::abs(t1) > std::abs(t2))
                        l2 = (lp->ls_3d.beg() + lp->ls_3d.direction() * (lp->ls_3d.beg().x() - crossing.x() * lp->ls_3d.beg().z()) / t1).lengthSquared();
                    else
                        l2 = (lp->ls_3d.beg() + lp->ls_3d.direction() * (lp->ls_3d.beg().y() - crossing.y() * lp->ls_3d.beg().z()) / t2).lengthSquared();

                    if (l1 > l2)
                        above = std::move(lp);
                    else
                        under = std::move(lp);
                }
                else
                    under = std::move(lp);
            }

            LineSegment3f ls_3d;
            LineSegment2f proj_2d;
            std::string style;
        };

        struct PolygonPrimitive : public RenderPrimitive
        {
            PolygonPrimitive() = default;
            PolygonPrimitive(const Polygon3Df &poly, std::string front_st, std::string back_st) : poly_3d{poly}, front_style{std::move(front_st)}, back_style{std::move(back_st)} {}
            PolygonPrimitive(const PolygonPrimitive &pp, bool with_points = true) : poly_3d{pp.poly_3d.normal(), pp.poly_3d.distance()}, front_style{pp.front_style}, back_style{pp.back_style}, front_visible{pp.front_visible}
            {
                if (with_points)
                {
                    poly_3d.addPointsDirect(pp.poly_3d.points().begin(), pp.poly_3d.points().end());
                    proj_2d.addPoints(pp.proj_2d.points().begin(), pp.proj_2d.points().end());
                }
            }
            ~PolygonPrimitive() override = default;

            void project(const RigidTf3f &tr, float fl) override
            {
                poly_3d = tr(poly_3d);
                front_visible = poly_3d.normal().dot(poly_3d.points()[0]) < 0;
                proj_2d.reservePoints(poly_3d.points().size());
                for (const auto &p : poly_3d.points())
                    proj_2d.addPoint(Vector2f(-fl * p.x() / p.z(), -fl * p.y() / p.z()));
            }

            std::string render(float scale) override
            {
                std::string code = "\\fill[";
                if (front_visible)
                    code += front_style;
                else
                    code += back_style;
                code += "] ";

                for (const auto &p : proj_2d.points())
                    code += "(" + std::to_string(p.x() * scale) + "," + std::to_string(p.y() * scale) + ") -- ";
                code += "cycle;\n";

                return code;
            }

            PolygonPrimitive* ptr() override { return this; }
            bool frontVisible() override { return front_visible; }

            void splitSort(std::unique_ptr<MarkPrimitive> mp, std::unique_ptr<MarkPrimitive> &under, std::unique_ptr<MarkPrimitive> &above, [[maybe_unused]]float fl, float eps) override
            {
                float d = poly_3d.normal().dot(mp->pos_3d) - poly_3d.distance();
                if (d > -eps)
                    above = std::move(mp);
                else
                    under = std::move(mp);
            }

            void splitSort(std::unique_ptr<LinePrimitive> lp, std::unique_ptr<LinePrimitive> &under, std::unique_ptr<LinePrimitive> &above, float fl, float eps) override
            {
                float nd = poly_3d.normal().dot(lp->ls_3d.direction());
                if (std::abs(nd) < eps)
                {
                    float d = poly_3d.normal().dot((lp->ls_3d.beg() + lp->ls_3d.end()) / 2.0f) - poly_3d.distance();
                    if (d > -eps)
                        above = std::move(lp);
                    else
                        under = std::move(lp);
                }
                else
                {
                    float t = - (poly_3d.normal().dot(lp->ls_3d.beg()) + poly_3d.d()) / nd;
                    if (t > eps && t < lp->ls_3d.length() - eps)
                    {
                        Vector3f crossing = lp->ls_3d.beg() + t * lp->ls_3d.direction();
                        std::unique_ptr<LinePrimitive> ls_beg(new LinePrimitive(LineSegment3f(lp->ls_3d.beg(), crossing), lp->style));
                        ls_beg->proj_2d = LineSegment2f(-Vector2f(ls_beg->ls_3d.beg().x(), ls_beg->ls_3d.beg().y()) * fl / ls_beg->ls_3d.beg().z(),
                                                        -Vector2f(crossing.x(), crossing.y()) * fl / crossing.z());
                        std::unique_ptr<LinePrimitive> ls_end(new LinePrimitive(LineSegment3f(crossing, lp->ls_3d.end()), lp->style));
                        ls_end->proj_2d = LineSegment2f(ls_beg->proj_2d.end(), -Vector2f(ls_end->ls_3d.end().x(), ls_end->ls_3d.end().y()) * fl / ls_end->ls_3d.end().z());

                        if (poly_3d.normal().dot(lp->ls_3d.beg()) - poly_3d.distance() > 0)
                        {
                            above = std::move(ls_beg);
                            under = std::move(ls_end);
                        }
                        else
                        {
                            above = std::move(ls_end);
                            under = std::move(ls_beg);
                        }
                    }
                    else
                    {
                        float d = poly_3d.normal().dot((lp->ls_3d.beg() + lp->ls_3d.end()) / 2.0f) - poly_3d.distance();
                        if (d > 0)
                            above = std::move(lp);
                        else
                            under = std::move(lp);
                    }
                }
            }

            void splitSort(std::unique_ptr<PolygonPrimitive> pp, std::list<std::unique_ptr<PolygonPrimitive>> &under, std::list<std::unique_ptr<PolygonPrimitive>> &above, float fl, float eps) override
            {
                bool p_on_front = false, intersects = false;
                size_t i = 0;

                // find a point of pp more distant than eps and determine, on which side of this polygon it lies
                for (; i < pp->poly_3d.points().size(); i++)
                {
                    float d = poly_3d.normal().dot(pp->poly_3d.points()[i]) - poly_3d.distance();
                    if (std::abs(d) > eps)
                    {
                        p_on_front = d > 0;
                        break;
                    }
                }

                if (i == pp->poly_3d.points().size()) // if none distant enough point was found, take the mean and decide
                {
                    auto v_sum = Vector3f::zeros();
                    for (const auto &pt : pp->poly_3d.points())
                        v_sum += pt;
                    p_on_front = poly_3d.normal().dot(v_sum / pp->poly_3d.points().size()) - poly_3d.distance() > 0;
                }
                else // else search for intersections of planes
                {
                    if (p_on_front)
                    {
                        for (i++; i < pp->poly_3d.points().size(); i++)
                        {
                            if (poly_3d.normal().dot(pp->poly_3d.points()[i]) - poly_3d.distance() < -eps)
                            {
                                intersects = true;
                                break;
                            }
                        }
                    }
                    else
                    {
                        for (i++; i < pp->poly_3d.points().size(); i++)
                        {
                            if (poly_3d.normal().dot(pp->poly_3d.points()[i]) - poly_3d.distance() > eps)
                            {
                                intersects = true;
                                break;
                            }
                        }
                    }
                }

                if (!intersects)
                {
                    if (p_on_front)
                        above.push_back(std::move(pp));
                    else
                        under.push_back(std::move(pp));
                    return;
                }
                else
                {
                    struct BrPt
                    {
                        BrPt() = default;
                        BrPt(const Vector3f& p3, const Vector2f &p2, bool above) : pt3d(p3), pt2d(p2), on_above(above) {}
                        BrPt(const Vector3f& p3, const Vector2f &p2, float il_t) : pt3d(p3), pt2d(p2), t(il_t) {}

                        Vector3f pt3d;
                        Vector2f pt2d;
                        float t{std::numeric_limits<float>::quiet_NaN()};
                        bool on_above{false};
                        bool from_under{false}, from_above{false}, to_under{false}, to_above{false};
                        int next_poly_under{0}, next_poly_above{0};

                        [[nodiscard]] bool isOnBrLine() const { return !std::isnan(t); }
                    };

                    std::list<BrPt> with_br_pts;
                    std::map<float, std::list<BrPt>::iterator> br_pts;
                    BrPt end_br_pt, tmp_br_pt;
                    Vector3f br_pt_first_3d = Vector3f::nan();
                    Vector3f il_d = poly_3d.normal().cross(pp->poly_3d.normal()).normalized();

                    for (size_t j = pp->proj_2d.points().size() - 1, k = 0; k < pp->proj_2d.points().size(); j = k, k++)
                    {
                        // if a point of pp lies on *this, it lies on intersection line as well
                        float j_to_il = poly_3d.normal().dot(pp->poly_3d.points()[j]) + poly_3d.d();

                        // if j-th point of pp lies on the intersection line, add it as a break point
                        if (std::abs(j_to_il) < eps)
                        {
                            float t = 0.0f;
                            if (br_pt_first_3d.hasNaN())
                                br_pt_first_3d = pp->poly_3d.points()[j];
                            else
                                t = (pp->poly_3d.points()[j] - br_pt_first_3d).dot(il_d);
                            with_br_pts.emplace_back(pp->poly_3d.points()[j], pp->proj_2d.points()[j], t);
                            br_pts[t] = --with_br_pts.end();
                        }
                        else
                        {
                            with_br_pts.emplace_back(pp->poly_3d.points()[j], pp->proj_2d.points()[j], j_to_il > 0);

                            // if k-th point of pp is not on the intersection line, check if j-to-k line segment crosses il
                            if(std::abs(poly_3d.normal().dot(pp->poly_3d.points()[k]) + poly_3d.d()) > eps)
                            {
                                Vector3f tmp_v3d = pp->poly_3d.points()[k] - pp->poly_3d.points()[j];
                                float t_ls_p = -(poly_3d.normal().dot(pp->poly_3d.points()[j]) + poly_3d.d()) / poly_3d.normal().dot(tmp_v3d);

                                if (t_ls_p > 0.0f && t_ls_p < 1.0f)
                                {
                                    tmp_v3d = pp->poly_3d.points()[j] + tmp_v3d * t_ls_p;

                                    float t = 0.0f;
                                    if (br_pt_first_3d.hasNaN())
                                        br_pt_first_3d = tmp_v3d;
                                    else
                                        t = (tmp_v3d - br_pt_first_3d).dot(il_d);
                                    with_br_pts.emplace_back(tmp_v3d, -Vector2f(tmp_v3d.x(), tmp_v3d.y()) * fl / tmp_v3d.z(), t);
                                    br_pts[t] = --with_br_pts.end();
                                }
                            }
                        }
                    }

                    for (auto k = std::prev(with_br_pts.end()), j = std::prev(k), l = with_br_pts.begin(); l != with_br_pts.end(); j = k, k = l, l++)
                    {
                        if (!j->isOnBrLine())
                        {
                            if (j->on_above)
                                k->from_above = true;
                            else
                                k->from_under = true;
                        }
                        if (!l->isOnBrLine())
                        {
                            if (l->on_above)
                                k->to_above = true;
                            else
                                k->to_under = true;
                        }
                    }

                    int npa = 0, npu = 0;
                    for (const auto &bp : br_pts)
                    {
                        if (bp.second->to_above)
                            npa++;
                        if (bp.second->to_under)
                            npu++;
                        if (bp.second->from_above)
                            npa--;
                        if (bp.second->from_under)
                            npu--;
                        bp.second->next_poly_under = npu;
                        bp.second->next_poly_above = npa;
                    }

                    auto increment_circular = [&with_br_pts](std::list<BrPt>::iterator &it)
                            {
                                it++;
                                if (it == with_br_pts.end())
                                    it = with_br_pts.begin();
                            };

                    for (const auto &bp : br_pts)
                    {
                        if (bp.second->to_above || bp.second->to_under)
                        {
                            std::unique_ptr<PolygonPrimitive> pp_new(new PolygonPrimitive(*pp, false));
                            auto first_bp = bp.second;
                            auto first_bp_to_above = first_bp->to_above;
                            auto pt_to_add = first_bp;
                            decltype(pt_to_add) previous_pt_to_add;

                            do
                            {
                                pp_new->proj_2d.addPoint(pt_to_add->pt2d);
                                pp_new->poly_3d.addPointDirect(pt_to_add->pt3d);

                                if (pt_to_add->isOnBrLine())
                                {
                                    if (pt_to_add->to_above && first_bp_to_above)
                                    {
                                        pt_to_add->to_above = false;
                                        previous_pt_to_add = pt_to_add;
                                        increment_circular(pt_to_add);
                                    }
                                    else if (pt_to_add->to_under && !first_bp_to_above)
                                    {
                                        pt_to_add->to_under = false;
                                        previous_pt_to_add = pt_to_add;
                                        increment_circular(pt_to_add);
                                    }
                                    else if (((pt_to_add->next_poly_above != 0 && first_bp_to_above) ||
                                            (pt_to_add->next_poly_under != 0 && !first_bp_to_above)) &&
                                            (previous_pt_to_add != (std::next(br_pts.find(pt_to_add->t)))->second))
                                    {
                                        previous_pt_to_add = pt_to_add;
                                        pt_to_add = (std::next(br_pts.find(pt_to_add->t)))->second;
                                    }
                                    else
                                    {
                                        previous_pt_to_add = pt_to_add;
                                        pt_to_add = (std::prev(br_pts.find(pt_to_add->t)))->second;
                                    }
                                }
                                else
                                {
                                    previous_pt_to_add = pt_to_add;
                                    increment_circular(pt_to_add);
                                }
                            } while (pt_to_add != first_bp);

                            if (first_bp_to_above)
                                above.push_back(std::move(pp_new));
                            else
                                under.push_back(std::move(pp_new));
                        }
                    }
                }
            }

            Polygon3Df poly_3d;
            Polygon2Df proj_2d;
            std::string front_style, back_style;
            bool front_visible{true};
        };

        struct RenderObj
        {
            virtual ~RenderObj() = default;
            virtual void primitives(std::list<std::unique_ptr<MarkPrimitive>> &mp, std::list<std::unique_ptr<LinePrimitive>> &lp, std::list<std::unique_ptr<PolygonPrimitive>> &pp) = 0;
        };

        struct FixedObj : public RenderObj
        {
            ~FixedObj() override = default;
            virtual BoundingBox3f boundingBox() = 0;
        };

        struct Mark : public FixedObj
        {
            Mark(const Vector3f &pos, std::string mn, float rot, float rad) : mark_name{std::move(mn)}, position{pos}, rotation{rot}, radius{rad} {}
            ~Mark() override = default;

            std::string mark_name;
            Vector3f position;
            float rotation, radius;

            void primitives(std::list<std::unique_ptr<MarkPrimitive>> &mp, [[maybe_unused]]std::list<std::unique_ptr<LinePrimitive>> &lp, [[maybe_unused]]std::list<std::unique_ptr<PolygonPrimitive>> &pp) override
            {
                mp.emplace_back(std::make_unique<LaTeXTikz3D::MarkPrimitive>(position, mark_name, rotation, radius));
            }

            BoundingBox3f boundingBox() override
            {
                return BoundingBox3f(position);
            }
        };

        struct Edge : public FixedObj
        {
            Edge(std::string sn, const Vector3f &beg, const Vector3f &end) : style_name{std::move(sn)}, axis{beg, end} {}
            Edge(std::string sn, const LineSegment3f &ls) : style_name{std::move(sn)}, axis{ls} {}
            ~Edge() override = default;

            std::string style_name;
            LineSegment3f axis;

            void primitives([[maybe_unused]]std::list<std::unique_ptr<MarkPrimitive>> &mp, std::list<std::unique_ptr<LinePrimitive>> &lp, [[maybe_unused]]std::list<std::unique_ptr<PolygonPrimitive>> &pp) override
            {
                lp.emplace_back(std::make_unique<LaTeXTikz3D::LinePrimitive>(axis, style_name));
            }

            BoundingBox3f boundingBox() override
            {
                return BoundingBox3f(axis.beg(), axis.end());
            }
        };

        struct Face : public FixedObj
        {
            Face(const Polygon3Df &poly, std::string front, std::string back, std::string line) : front_style_name{std::move(front)}, back_style_name{std::move(back)},
                                                                                                    line_style_name{std::move(line)}, polygon{poly} {}
            ~Face() override = default;

            std::string front_style_name, back_style_name, line_style_name;
            Polygon3Df polygon;

            void primitives([[maybe_unused]]std::list<std::unique_ptr<MarkPrimitive>> &mp, std::list<std::unique_ptr<LinePrimitive>> &lp, std::list<std::unique_ptr<PolygonPrimitive>> &pp) override
            {
                if (!front_style_name.empty() || !back_style_name.empty())
                    pp.emplace_back(std::make_unique<LaTeXTikz3D::PolygonPrimitive>(polygon, front_style_name, back_style_name));
                if (!line_style_name.empty())
                {
                    size_t pts_cnt = polygon.points().size();
                    for (size_t i = pts_cnt - 1, j = 0; j < pts_cnt; i = j, j++)
                        lp.emplace_back(std::make_unique<LaTeXTikz3D::LinePrimitive>(LineSegment3f(polygon.points()[i], polygon.points()[j]), line_style_name));
                }
            }

            BoundingBox3f boundingBox() override
            {
                BoundingBox3f bb(polygon.points()[0]);
                for (const auto &p : polygon.points())
                    bb.addPoint(p);
                return bb;
            }
        };

        struct AdaptingObj : public RenderObj
        {
            ~AdaptingObj() override = default;
            virtual void fitTo(const BoundingBox3f &) = 0;
        };

        struct Grid : public AdaptingObj
        {
            Grid(size_t ax, std::string sn, float t) : style_name{std::move(sn)}, axis{ax}, tick{t} {}
            ~Grid() override = default;
            std::string style_name;
            size_t axis{};
            float tick{};

            void primitives([[maybe_unused]]std::list<std::unique_ptr<MarkPrimitive>> &mp, [[maybe_unused]]std::list<std::unique_ptr<LinePrimitive>> &lp, [[maybe_unused]]std::list<std::unique_ptr<PolygonPrimitive>> &pp) override
            {

            }

            void fitTo([[maybe_unused]]const BoundingBox3f &bb) override
            {

            }
        };

        struct Axis : public AdaptingObj
        {
            Axis(std::string sn, std::string nf, unsigned int np, float t, const Vector3f &beg, const Vector3f &end) : style_name{std::move(sn)}, num_format{std::move(nf)},
                                                                                                                       num_position{np}, tick{t}, axis{beg, end} {}
            ~Axis() override = default;
            std::string style_name, num_format;
            unsigned int num_position;
            float tick;
            LineSegment3f axis;

            void primitives([[maybe_unused]]std::list<std::unique_ptr<MarkPrimitive>> &mp, std::list<std::unique_ptr<LinePrimitive>> &lp, [[maybe_unused]]std::list<std::unique_ptr<PolygonPrimitive>> &pp) override
            {
                lp.emplace_back(std::make_unique<LaTeXTikz3D::LinePrimitive>(axis, style_name));
            }

            void fitTo(const BoundingBox3f &bb) override
            {
                axis.fitToHyperRect(bb.min(), bb.max());
            }
        };

        std::map<std::string, std::string> styles;
        std::map<std::string, std::string> marks;
        std::map<std::string, std::string> colors;
        std::list<std::unique_ptr<AdaptingObj>> adapting_objects;
        std::list<std::unique_ptr<FixedObj>> fixed_objects;

        std::list<std::unique_ptr<MarkPrimitive>> mark_primitives;
        std::list<std::unique_ptr<LinePrimitive>> line_primitives;
        std::list<std::unique_ptr<PolygonPrimitive>> polygon_primitives;
        std::list<std::unique_ptr<RenderPrimitive>> render_primitives;

        float epsilon{0.001f};
        float export_width{}, export_height{}, export_border{}, focal_length{};
        RigidTf3f view_orientation;
        std::unique_ptr<BoundingBox3f> min_reg{nullptr}, max_reg{nullptr}, render_reg{nullptr};
        std::unique_ptr<BoundingBox2f> clipping{nullptr};
        std::string frame_style;

        // axes + grids
        unsigned int overrun_type{};
        float overrun_magnitude{};

        std::string saveStyle(const std::string &style)
        {
            if (style.empty())
                return std::string();

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

        std::string saveMark(const std::string &style_name, const std::string &mark_template, float mark_radius)
        {
            std::string mark_code = "\n\t\\begin{scope}[style=" + style_name +
                                    ",xshift=#1cm,yshift=#2cm,rotate=#3]\n\t\\pgfmathsetmacro{\\MarkRadius}{" +
                                    std::to_string(mark_radius) + "*#4}\n\t" + mark_template + "\n\t\\end{scope}";

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
            if (pos & LaTeXTikz3D::position_above)
                str += "south ";
            else if (pos & LaTeXTikz3D::position_below)
                str += "north ";
            if (pos & LaTeXTikz3D::position_right)
                str += "west";
            else if (pos & LaTeXTikz3D::position_left)
                str += "east";
            return str;
        }
    };
}

#endif //ROBOTICTEMPLATELIBRARY_IO_LATEXTIKZ3D_H
