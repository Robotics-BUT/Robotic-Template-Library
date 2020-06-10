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

#include "rtl/Core.h"
#include "rtl/Transformation.h"

namespace rtl
{
    //! LaTeX export of high quality vector graphics using the Tikz package - 3D scene rendering into 2D drawing.
    /*!
     * This class is used to aggregate graphic primitives to be rendered into a PDF format. The rendering order is determined using traditional visibility testing and do not require
     * any input from the user side. View of the scene can be set using regular translation-rotation-projection scheme.
     */
    class LaTeXTikz3D
    {
    public:
        //! Default constructor with basic initialization of the exporter.
        LaTeXTikz3D();

        //! Move constructor.
        LaTeXTikz3D(LaTeXTikz3D &&le) noexcept;

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
        void setBorder(float border) { export_border = border; }

        //! Enables and sets style of a frame around the exported figure.
        /*!
         *
         * @param style Tikz style of the frame.
         */
        void setFrameStyle(const std::string &style) { frame_style = style; }

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
        void clearAll();

        //! Clears only data, while export settings are left unchanged.
        void clearData();

        //! Writes internal data according to export settings into .tex file.
        /*!
         *
         * @param file_name output file path.
         */
        void writeTEX(const std::string &file_name);

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
        void addAxis(const std::string &style, std::string num_format, unsigned int num_position, float tick, const Vector3f &beg, const Vector3f &end);

        //! Adds a mark to the rendering.
        /*!
         *
         * @param pos position of the mark in space.
         * @param mark_style Tikz style.
         * @param mark_template Tikz drawing code of the mark.
         * @param rotation rotation of he mark in the projection plane.
         * @param mark_radius individual scaling of the mark.
         */
        void addMark(const Vector3f &pos, const std::string &mark_style, const std::string &mark_template, float rotation = 0.0f, float mark_radius = 1.0f);

        //! Adds multiple marks to the rendering.
        /*!
         *
         * @param vpos positions of the marks.
         * @param mark_style Tikz style.
         * @param mark_template Tikz drawing code of the mark.
         * @param rotation rotation of he mark in the projection plane.
         * @param mark_radius individual scaling of the mark.
         */
        void addMarks(const std::vector<Vector3f> &vpos, const std::string &mark_style, const std::string &mark_template, float rotation = 0.0f, float mark_radius = 1.0f);

        //! Adds a line to the rendering.
        /*!
         *
         * @param ls line segment.
         * @param style Tikz style.
         */
        void addLine(const LineSegment3f &ls, const std::string &style);

        //! Adds multiple lines to the rendering.
        /*!
         *
         * @param vls line segment to be rendered.
         * @param style Tikz style.
         */
        void addLines(const std::vector<LineSegment3f> &vls, const std::string &style);

        //! Adds a polygonal face to the rendering.
        /*!
         *
         * @param face polygonal face.
         * @param front_style Tikz style of the front side (the side from which the normal vector origins).
         * @param back_style Tikz style of the front side.
         * @param line_style Tikz style of the outline.
         */
        void addFace(const Polygon3Df &face, const std::string &front_style, const std::string &back_style, const std::string &line_style);

        //! Store given color.
        /*!
         *
         * @param color Tikz compliant color specification.
         * @return code for referencing the \p color in .tex files.
         */
        std::string saveColor(const std::string &color);

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

            void project(const RigidTf3f &tr, float fl) override;
            std::string render(float scale) override;
            MarkPrimitive* ptr() override { return this; }
            bool frontVisible() override { return true; }

            void splitSort(std::unique_ptr<MarkPrimitive> mp, std::unique_ptr<MarkPrimitive> &under, std::unique_ptr<MarkPrimitive> &above, float fl, float eps) override;

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

            void project(const RigidTf3f &tr, float fl) override;
            std::string render(float scale) override;
            LinePrimitive* ptr() override { return this; }
            bool frontVisible() override { return true; }

            void splitSort(std::unique_ptr<MarkPrimitive> mp, std::unique_ptr<MarkPrimitive> &under, std::unique_ptr<MarkPrimitive> &above, float fl, float eps) override;
            void splitSort(std::unique_ptr<LinePrimitive> lp, std::unique_ptr<LinePrimitive> &under, std::unique_ptr<LinePrimitive> &above, float fl, float eps) override;

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

            void project(const RigidTf3f &tr, float fl) override;
            std::string render(float scale) override;
            PolygonPrimitive* ptr() override { return this; }
            bool frontVisible() override { return front_visible; }

            void splitSort(std::unique_ptr<MarkPrimitive> mp, std::unique_ptr<MarkPrimitive> &under, std::unique_ptr<MarkPrimitive> &above, float fl, float eps) override;
            void splitSort(std::unique_ptr<LinePrimitive> lp, std::unique_ptr<LinePrimitive> &under, std::unique_ptr<LinePrimitive> &above, float fl, float eps) override;
            void splitSort(std::unique_ptr<PolygonPrimitive> pp, std::list<std::unique_ptr<PolygonPrimitive>> &under, std::list<std::unique_ptr<PolygonPrimitive>> &above, float fl, float eps) override;

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

            void primitives(std::list<std::unique_ptr<MarkPrimitive>> &mp, std::list<std::unique_ptr<LinePrimitive>> &lp, std::list<std::unique_ptr<PolygonPrimitive>> &pp) override;
            BoundingBox3f boundingBox() override;
        };

        struct Edge : public FixedObj
        {
            Edge(std::string sn, const Vector3f &beg, const Vector3f &end) : style_name{std::move(sn)}, axis{beg, end} {}
            Edge(std::string sn, const LineSegment3f &ls) : style_name{std::move(sn)}, axis{ls} {}
            ~Edge() override = default;

            std::string style_name;
            LineSegment3f axis;

            void primitives(std::list<std::unique_ptr<MarkPrimitive>> &mp, std::list<std::unique_ptr<LinePrimitive>> &lp, std::list<std::unique_ptr<PolygonPrimitive>> &pp) override;
            BoundingBox3f boundingBox() override;
        };

        struct Face : public FixedObj
        {
            Face(const Polygon3Df &poly, std::string front, std::string back, std::string line) : front_style_name{std::move(front)}, back_style_name{std::move(back)},
                                                                                                    line_style_name{std::move(line)}, polygon{poly} {}
            ~Face() override = default;

            std::string front_style_name, back_style_name, line_style_name;
            Polygon3Df polygon;

            void primitives(std::list<std::unique_ptr<MarkPrimitive>> &mp, std::list<std::unique_ptr<LinePrimitive>> &lp, std::list<std::unique_ptr<PolygonPrimitive>> &pp) override;
            BoundingBox3f boundingBox() override;
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

            void primitives(std::list<std::unique_ptr<MarkPrimitive>> &mp, std::list<std::unique_ptr<LinePrimitive>> &lp, std::list<std::unique_ptr<PolygonPrimitive>> &pp) override;
            void fitTo(const BoundingBox3f &bb) override;
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

            void primitives(std::list<std::unique_ptr<MarkPrimitive>> &mp, std::list<std::unique_ptr<LinePrimitive>> &lp, std::list<std::unique_ptr<PolygonPrimitive>> &pp) override;
            void fitTo(const BoundingBox3f &bb) override;
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

        std::string saveStyle(const std::string &style);

        std::string saveMark(const std::string &style_name, const std::string &mark_template, float mark_radius);

        static std::string digitsToLetters(size_t num);

        static std::string positionToAnchor(unsigned int pos);
    };
}

#endif //ROBOTICTEMPLATELIBRARY_IO_LATEXTIKZ3D_H
