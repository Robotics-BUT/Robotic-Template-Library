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

#include "rtl/io/LaTeXTikz3D.h"

#include <fstream>
#include <algorithm>
#include <iostream>

namespace rtl
{
    const unsigned int LaTeXTikz3D::overrun_relative = 0;
    const unsigned int LaTeXTikz3D::overrun_absolute = 1;

    const unsigned int LaTeXTikz3D::axis_type_linear = 0;
    const unsigned int LaTeXTikz3D::axis_type_log10 = 1;

    const unsigned int LaTeXTikz3D::position_above = 0x01;
    const unsigned int LaTeXTikz3D::position_below = 0x02;
    const unsigned int LaTeXTikz3D::position_right = 0x04;
    const unsigned int LaTeXTikz3D::position_left = 0x08;

    const char *LaTeXTikz3D::latex_mark_blank = "";
    const char *LaTeXTikz3D::latex_mark_cross = "\\draw (-\\MarkRadius,-\\MarkRadius) -- (\\MarkRadius,\\MarkRadius);\n\t\\draw (\\MarkRadius,-\\MarkRadius) -- (-\\MarkRadius, \\MarkRadius);";
    const char *LaTeXTikz3D::latex_mark_mark = "\\draw (0,-\\MarkRadius) -- (0,\\MarkRadius);";
    const char *LaTeXTikz3D::latex_mark_dot = "\\fill (0, 0) circle [radius=\\MarkRadius];\n\t\\draw (0, 0) circle [radius=\\MarkRadius];";
    const char *LaTeXTikz3D::latex_mark_robot = "\\fill (0, 0) circle [radius=\\MarkRadius];\n\t\\draw (0, 0) circle [radius=\\MarkRadius];\n\t\\draw (0, 0) -- (2*\\MarkRadius,0);";

    LaTeXTikz3D::LaTeXTikz3D()
    {
        clearAll();
    }

    LaTeXTikz3D::LaTeXTikz3D(LaTeXTikz3D &&le) noexcept
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

    void LaTeXTikz3D::clearAll()
    {
        clearData();
        export_width = export_height = 10.0f;
        export_border = 0.1f;
        min_reg = nullptr;
        max_reg = nullptr;
        frame_style.clear();
    }

    void LaTeXTikz3D::clearData()
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

    std::string LaTeXTikz3D::saveStyle(const std::string &style)
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

    std::string LaTeXTikz3D::saveMark(const std::string &style_name, const std::string &mark_template, float mark_radius)
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

    std::string LaTeXTikz3D::saveColor(const std::string &color)
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

    std::string LaTeXTikz3D::digitsToLetters(size_t num)
    {
        char buffer[50];
        int l = snprintf(buffer, sizeof(buffer), "%zd", num);
        if (l < 0)
            return std::string();
        for (int i = 0; i < l; i++)
            buffer[i] += 17;
        return std::string(buffer);
    }

    std::string LaTeXTikz3D::positionToAnchor(unsigned int pos)
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

    void LaTeXTikz3D::writeTEX(const std::string &file_name)
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
                {
                    render_primitives.insert(rp_rit.base(), std::move(labove));
                    goto new_line;
                }
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

    void LaTeXTikz3D::addAxis(const std::string &style, std::string num_format, unsigned int num_position, float tick, const Vector3f &beg, const Vector3f &end)
    {
        adapting_objects.push_back(std::make_unique<Axis>(saveStyle(style), std::move(num_format), num_position, tick, beg, end));
    }

    void LaTeXTikz3D::addMark(const Vector3f &pos, const std::string &mark_style, const std::string &mark_template, float rotation, float mark_radius)
    {
        fixed_objects.push_back(std::make_unique<Mark>(pos, saveMark(saveStyle(mark_style), mark_template, mark_radius), rotation, mark_radius));
    }

    void LaTeXTikz3D::addMarks(const std::vector<Vector3f> &vpos, const std::string &mark_style, const std::string &mark_template, float rotation, float mark_radius)
    {
        std::string mark_key = saveMark(saveStyle(mark_style), mark_template, mark_radius);
        for (const Vector3f &v : vpos)
            fixed_objects.push_back(std::make_unique<Mark>(v, mark_key, rotation, mark_radius));
    }

    void LaTeXTikz3D::addLine(const LineSegment3f &ls, const std::string &style)
    {
        fixed_objects.push_back(std::make_unique<Edge>(saveStyle(style), ls));
    }

    void LaTeXTikz3D::addLines(const std::vector<LineSegment3f> &vls, const std::string &style)
    {
        std::string style_key = saveStyle(style);
        for (const LineSegment3f &ls : vls)
            fixed_objects.push_back(std::make_unique<Edge>(style_key, ls));
    }

    void LaTeXTikz3D::addFace(const Polygon3Df &face, const std::string &front_style, const std::string &back_style, const std::string &line_style)
    {
        if (face.points().size() > 2)
            fixed_objects.push_back(std::make_unique<Face>(face, saveStyle(front_style), saveStyle(back_style), saveStyle(line_style)));
    }

    void LaTeXTikz3D::Grid::primitives(std::list<std::unique_ptr<MarkPrimitive>> &, std::list<std::unique_ptr<LinePrimitive>> &, std::list<std::unique_ptr<PolygonPrimitive>> &)
    {

    }

    void LaTeXTikz3D::Grid::fitTo(const BoundingBox3f &)
    {

    }

    void LaTeXTikz3D::Axis::primitives(std::list<std::unique_ptr<MarkPrimitive>> &, std::list<std::unique_ptr<LinePrimitive>> &lp, std::list<std::unique_ptr<PolygonPrimitive>> &)
    {
        lp.emplace_back(std::make_unique<LaTeXTikz3D::LinePrimitive>(axis, style_name));
    }

    void LaTeXTikz3D::Axis::fitTo(const BoundingBox3f &bb)
    {
        axis.fitToHyperRect(bb.min(), bb.max());
    }

    std::string LaTeXTikz3D::MarkPrimitive::render(float sc)
    {
        return "\t\\" + mark + "{" + std::to_string(proj_2d.x() * sc) + "}{" + std::to_string(proj_2d.y() * sc) + "}{" + std::to_string(rotation) + "}{" + std::to_string(scale * sc) + "}\n";
    }

    void LaTeXTikz3D::MarkPrimitive::project(const RigidTf3f &tr, float fl)
    {
        pos_3d = tr(pos_3d);
        scale = -fl / pos_3d.z();
        proj_2d = Vector2f(pos_3d.x(), pos_3d.y()) * scale;
    }

    void LaTeXTikz3D::MarkPrimitive::splitSort(std::unique_ptr<LaTeXTikz3D::MarkPrimitive> mp, std::unique_ptr<LaTeXTikz3D::MarkPrimitive> &under, std::unique_ptr<LaTeXTikz3D::MarkPrimitive> &above, float, float)
    {
        if (pos_3d.lengthSquared() > mp->pos_3d.lengthSquared())
            above = std::move(mp);
        else
            under = std::move(mp);
    }

    std::string LaTeXTikz3D::LinePrimitive::render(float scale)
    {
        return "\t\\draw[" + style + "] (" + std::to_string(proj_2d.beg().x() * scale) + "," + std::to_string(proj_2d.beg().y() * scale) +
               ") -- (" + std::to_string(proj_2d.end().x() * scale) + "," + std::to_string(proj_2d.end().y() * scale) + ");\n";
    }

    void LaTeXTikz3D::LinePrimitive::project(const RigidTf3f &tr, float fl)
    {
        ls_3d = tr(ls_3d);
        proj_2d.setBegin(-fl * ls_3d.beg().x() / ls_3d.beg().z(), -fl * ls_3d.beg().y() / ls_3d.beg().z());
        proj_2d.setEnd(-fl * ls_3d.end().x() / ls_3d.end().z(), -fl * ls_3d.end().y() / ls_3d.end().z());
    }

    void LaTeXTikz3D::LinePrimitive::splitSort(std::unique_ptr<LaTeXTikz3D::MarkPrimitive> mp, std::unique_ptr<LaTeXTikz3D::MarkPrimitive> &under, std::unique_ptr<LaTeXTikz3D::MarkPrimitive> &above, float fl, float eps)
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

    void LaTeXTikz3D::LinePrimitive::splitSort(std::unique_ptr<LaTeXTikz3D::LinePrimitive> lp, std::unique_ptr<LaTeXTikz3D::LinePrimitive> &under, std::unique_ptr<LaTeXTikz3D::LinePrimitive> &above, float, float eps)
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

    std::string LaTeXTikz3D::PolygonPrimitive::render(float scale)
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

    void LaTeXTikz3D::PolygonPrimitive::project(const RigidTf3f &tr, float fl)
    {
        poly_3d = tr(poly_3d);
        front_visible = poly_3d.normal().dot(poly_3d.points()[0]) < 0;
        proj_2d.reservePoints(poly_3d.points().size());
        for (const auto &p : poly_3d.points())
            proj_2d.addPoint(Vector2f(-fl * p.x() / p.z(), -fl * p.y() / p.z()));
    }

    void LaTeXTikz3D::PolygonPrimitive::splitSort(std::unique_ptr<LaTeXTikz3D::MarkPrimitive> mp, std::unique_ptr<LaTeXTikz3D::MarkPrimitive> &under, std::unique_ptr<LaTeXTikz3D::MarkPrimitive> &above, float, float eps)
    {
        float d = poly_3d.normal().dot(mp->pos_3d) - poly_3d.distance();
        if (d > -eps)
            above = std::move(mp);
        else
            under = std::move(mp);
    }
    
    void LaTeXTikz3D::PolygonPrimitive::splitSort(std::unique_ptr<LaTeXTikz3D::LinePrimitive> lp, std::unique_ptr<LaTeXTikz3D::LinePrimitive> &under, std::unique_ptr<LaTeXTikz3D::LinePrimitive> &above, float fl, float eps)
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
    
    void LaTeXTikz3D::PolygonPrimitive::splitSort(std::unique_ptr<LaTeXTikz3D::PolygonPrimitive> pp, std::list<std::unique_ptr<LaTeXTikz3D::PolygonPrimitive>> &under, std::list<std::unique_ptr<LaTeXTikz3D::PolygonPrimitive>> &above, float fl, float eps)
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
                bool to_above{false};
                float t{0.0f}, et{std::numeric_limits<float>::quiet_NaN()};
                size_t real_idx{static_cast<size_t>(-1)};
                Vector3f pt3d;
                Vector2f pt2d;
                std::list<size_t> poly_pts;
            };

            std::map<float, BrPt> br_pts;
            BrPt end_br_pt, tmp_br_pt;
            Vector3f br_pt_first_3d;
            Vector3f il_d = poly_3d.normal().cross(pp->poly_3d.normal()).normalized();

            for (size_t j = pp->proj_2d.points().size() - 1, k = 0; k < pp->proj_2d.points().size(); j = k, k++)
            {
                bool j_on_il = std::abs(poly_3d.normal().dot(pp->poly_3d.points()[j]) + poly_3d.d()) < eps;
                bool k_on_il = std::abs(poly_3d.normal().dot(pp->poly_3d.points()[k]) + poly_3d.d()) < eps;

                if (j_on_il)
                {
                    if (k_on_il)
                    {
                        if (std::isnan(tmp_br_pt.et))
                            br_pt_first_3d = pp->poly_3d.points()[k];
                        float bt = (std::isnan(tmp_br_pt.et)) ? 0.0f : (pp->poly_3d.points()[k] - br_pt_first_3d).dot(il_d);

                        tmp_br_pt.et = bt;
                        if (bt == 0.0f)
                            end_br_pt = tmp_br_pt;
                        else
                            br_pts[tmp_br_pt.t] = tmp_br_pt;

                        tmp_br_pt.poly_pts.clear();
                        tmp_br_pt.pt2d = pp->proj_2d.points()[k];
                        tmp_br_pt.pt3d = pp->poly_3d.points()[k];
                        tmp_br_pt.t = bt;
                    }
                    else
                    {
                        tmp_br_pt.poly_pts.push_back(k);
                    }
                }
                else
                {
                    if(k_on_il)
                    {
                        if (std::isnan(tmp_br_pt.et))
                            br_pt_first_3d = pp->poly_3d.points()[k];
                        float bt = (std::isnan(tmp_br_pt.et)) ? 0.0f : (pp->poly_3d.points()[k] - br_pt_first_3d).dot(il_d);

                        tmp_br_pt.et = bt;
                        if (bt == 0.0f)
                            end_br_pt = tmp_br_pt;
                        else
                            br_pts[tmp_br_pt.t] = tmp_br_pt;

                        tmp_br_pt.poly_pts.clear();
                        tmp_br_pt.pt2d = pp->proj_2d.points()[k];
                        tmp_br_pt.pt3d = pp->poly_3d.points()[k];
                        tmp_br_pt.t = bt;
                    }
                    else
                    {
                        Vector3f tmp_v3d = pp->poly_3d.points()[k] - pp->poly_3d.points()[j];
                        float t_ls_p = -(poly_3d.normal().dot(pp->poly_3d.points()[j]) + poly_3d.d()) / poly_3d.normal().dot(tmp_v3d);

                        if (t_ls_p > 0.0f && t_ls_p < 1.0f)
                        {
                            tmp_v3d = pp->poly_3d.points()[j] + tmp_v3d * t_ls_p;
                            if (std::isnan(tmp_br_pt.et))
                                br_pt_first_3d = tmp_v3d;
                            float bt = (std::isnan(tmp_br_pt.et)) ? 0.0f : (tmp_v3d - br_pt_first_3d).dot(il_d);

                            tmp_br_pt.et = bt;
                            if (bt == 0.0f)
                                end_br_pt = tmp_br_pt;
                            else
                                br_pts[tmp_br_pt.t] = tmp_br_pt;

                            tmp_br_pt.poly_pts.clear();
                            tmp_br_pt.pt2d = -Vector2f(tmp_v3d.x(), tmp_v3d.y()) * fl / tmp_v3d.z();
                            tmp_br_pt.pt3d = tmp_v3d;
                            tmp_br_pt.t = bt;
                        }
                        tmp_br_pt.poly_pts.push_back(k);
                    }
                }
            }
            tmp_br_pt.et = end_br_pt.et;
            tmp_br_pt.poly_pts.splice(tmp_br_pt.poly_pts.end(), end_br_pt.poly_pts);
            br_pts[tmp_br_pt.t] = tmp_br_pt;

            for (auto it = br_pts.begin(); it != br_pts.end(); it++)
            {
                if (!it->second.poly_pts.empty())
                {
                    for (auto end_it = br_pts.find(it->second.et); end_it->second.poly_pts.empty(); end_it = br_pts.find(it->second.et))
                    {
                        it->second.poly_pts.push_back(end_it->second.real_idx);
                        it->second.et = end_it->second.et;
                        br_pts.erase(end_it);
                    }
                }
            }

            for (auto &it_bp : br_pts)
            {
                auto pt_mean = Vector3f::zeros();
                for (auto pt_i : it_bp.second.poly_pts)
                    pt_mean += pp->poly_3d.points()[pt_i];
                pt_mean /= it_bp.second.poly_pts.size();
                it_bp.second.to_above = poly_3d.normal().dot(pt_mean) - poly_3d.distance() > 0;
            }

            bool it_even = true;
            auto grab_points = [&pp, &br_pts](std::unique_ptr<PolygonPrimitive> &to, BrPt &from)
                    {
                        to->proj_2d.addPoint(from.pt2d);
                        to->poly_3d.addPoint(from.pt3d);
                        for (auto idx : from.poly_pts)
                        {
                            to->proj_2d.addPoint(pp->proj_2d.points()[idx]);
                            to->poly_3d.addPointDirect(pp->poly_3d.points()[idx]);
                        }
                        auto from_end = br_pts.find(from.et);
                        to->proj_2d.addPoint(from_end->second.pt2d);
                        to->poly_3d.addPoint(from_end->second.pt3d);
                        from.poly_pts.clear();
                    };

            for (auto it_bp = br_pts.begin(); it_bp != br_pts.end();)
            {
                if (it_bp->second.t == it_bp->second.et)
                {
                    std::unique_ptr<PolygonPrimitive> pp_new(new PolygonPrimitive(*pp, false));
                    grab_points(pp_new, it_bp->second);
                    if (it_bp->second.to_above)
                        above.push_back(std::move(pp_new));
                    else
                        under.push_back(std::move(pp_new));
                    auto next = std::next(it_bp);
                    br_pts.erase(it_bp);
                    it_bp = next;
                }
                else
                    it_bp++;
            }

            if(br_pts.size() % 2 == 1)
                std::cout<<"Problem"<<std::endl;

            for (auto it = br_pts.begin(); it != br_pts.end(); it++, it_even = !it_even)
            {
                if (!it->second.poly_pts.empty())
                {
                    std::unique_ptr<PolygonPrimitive> pp_new(new PolygonPrimitive(*pp, false));
                    grab_points(pp_new, it->second);

                    auto following = (it_even) ? std::prev(br_pts.find(it->second.et)) : std::next(br_pts.find(it->second.et));
                    while (it->second.t != following->second.t)
                    {
                        grab_points(pp_new, following->second);
                        following = (it_even) ? std::prev(br_pts.find(following->second.et)) : std::next(br_pts.find(following->second.et));
                    }

                    if (it->second.to_above)
                        above.push_back(std::move(pp_new));
                    else
                        under.push_back(std::move(pp_new));
                }
            }
        }   
    }

    void LaTeXTikz3D::Edge::primitives(std::list<std::unique_ptr<MarkPrimitive>> &, std::list<std::unique_ptr<LinePrimitive>> &lp, std::list<std::unique_ptr<PolygonPrimitive>> &)
    {
        lp.emplace_back(std::make_unique<LaTeXTikz3D::LinePrimitive>(axis, style_name));
    }

    BoundingBox3f LaTeXTikz3D::Edge::boundingBox()
    {
        return BoundingBox3f(axis.beg(), axis.end());
    }

    void LaTeXTikz3D::Face::primitives(std::list<std::unique_ptr<MarkPrimitive>> &, std::list<std::unique_ptr<LinePrimitive>> &lp, std::list<std::unique_ptr<PolygonPrimitive>> &pp)
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

    BoundingBox3f LaTeXTikz3D::Face::boundingBox()
    {
        BoundingBox3f bb(polygon.points()[0]);
        for (const auto &p : polygon.points())
            bb.addPoint(p);
        return bb;
    }

    void LaTeXTikz3D::Mark::primitives(std::list<std::unique_ptr<MarkPrimitive>> &mp, std::list<std::unique_ptr<LinePrimitive>> &, std::list<std::unique_ptr<PolygonPrimitive>> &)
    {
        mp.emplace_back(std::make_unique<LaTeXTikz3D::MarkPrimitive>(position, mark_name, rotation, radius));
    }

    BoundingBox3f LaTeXTikz3D::Mark::boundingBox()
    {
        return BoundingBox3f(position);
    }
}