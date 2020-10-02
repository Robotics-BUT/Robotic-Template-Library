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
#include <string>
#include <string_view>
#include <iostream>

#include "rtl/Core.h"
#include "rtl/Transformation.h"
#include "rtl/Test.h"
#include "rtl/io/LaTeXDoc.h"


// global settings
std::string true_color = "green!60!white";
std::string false_color = "red!60!white";


// Adopted from https://stackoverflow.com/a/56766138
template <typename T>
constexpr auto typeName() noexcept
{
    std::string_view name, prefix, suffix;
#ifdef __clang__
    name = __PRETTY_FUNCTION__;
    prefix = "auto type_name() [T = ";
    suffix = "]";
#elif defined(__GNUC__)
    name = __PRETTY_FUNCTION__;
    prefix = "constexpr auto type_name() [with T = ";
    suffix = "]";
#elif defined(_MSC_VER)
    name = __FUNCSIG__;
    prefix = "auto __cdecl type_name<";
    suffix = ">(void) noexcept";
#endif
    name.remove_prefix(prefix.size()-1);
    name.remove_suffix(suffix.size());
    return name;
}

template<template<int, typename> class... Tfs>
struct TransformableTestImpl
{
    static const int tfs_cnt = sizeof...(Tfs);

    static std::vector<std::string> tfsNames()
    {
        static std::vector<std::string> names = {rtl::test::type<Tfs<3, float>>::description()...};
        return names;
    }

    template<typename T>
    static std::vector<bool> isTransformableValues()
    {
        if constexpr (rtl::has_element_type_v<T> && rtl::is_dimensional_v<T>)
        {
            static std::vector<bool> values = {rtl::is_transformable_v<T, Tfs<T::dimensionality(), typename T::ElementType>>...};
            return values;
        }
        else
        {
            static std::vector<bool> values(tfs_cnt, false);
            return values;
        }
    }
};

// Add new transformation to be tested here
using TransformableTest = TransformableTestImpl<rtl::TranslationND, rtl::RotationND, rtl::RigidTfND>;


template<template<typename> class... Trs>
struct TraitTestImpl
{
    static const int trs_cnt = sizeof...(Trs);

    static std::vector<std::string> trsNames()
    {
        static std::vector<std::string> names = {std::string(typeName<Trs<void>>())...};
        return names;
    }

    template<typename T>
    static std::vector<bool> traitsValues()
    {
        static std::vector<bool> values = {Trs<T>::value...};
        return values;
    }
};

template<typename T>
using has_random_single_arg = rtl::has_random<T>;

// Add new type traits to be tested here
using TraitTest = TraitTestImpl<rtl::is_dimensional, rtl::has_element_type, rtl::has_metric, rtl::is_invertible, rtl::has_identity, rtl::has_nan, has_random_single_arg>;



struct TypeData
{
    TypeData() = default;

    explicit TypeData(std::vector<TypeData> &&vtd)
    {
        traits_values = std::vector<bool>(TraitTest::trs_cnt, true);
        is_transformable_values = std::vector<bool>(TransformableTest ::tfs_cnt, true);
        description = vtd.front().description;
        for (const auto &td : vtd)
            merge(td);
    }

    void merge(const TypeData &td)
    {
        for (size_t i = 0; i < traits_values.size(); i++)
            traits_values[i] = traits_values[i] && td.traits_values[i];
        for (size_t i = 0; i < is_transformable_values.size(); i++)
            is_transformable_values[i] = is_transformable_values[i] && td.is_transformable_values[i];
    }

    std::string description;
    std::vector<bool> traits_values, is_transformable_values;
};

template<typename T>
TypeData examineType()
{
    TypeData td;
    td.description = rtl::test::type<T>::description();
    td.is_transformable_values = TransformableTest::template isTransformableValues<T>();
    td.traits_values = TraitTest::template traitsValues<T>();
    return td;
}

template<template<typename>class T, typename... Es>
TypeData examineTemplateTypes()
{
    static_assert(sizeof...(Es) != 0, "At least one element type must be specified for examineTemplateTypes().");
    return TypeData({examineType<T<Es>>()...});
}

template<template<int, typename>class Trt, int dim>
struct SpecializeRange1
{
    template<typename Tt>
    using spec = Trt<dim, Tt>;
};

template<template<int, typename>class T, int r1_min, typename... Es, typename I, I... ints>
TypeData examineTemplateRangeTypes([[maybe_unused]] std::integer_sequence<I, ints...> r1)
{
    static_assert(sizeof...(ints) > 0, "At least one dimension in r1 is required for examineTemplateRangeTypes().");
    return TypeData({examineTemplateTypes<SpecializeRange1<T, r1_min + ints>::template spec, Es...>()...});
}

template<template<int, int, typename>class Trt, int dim>
struct SpecializeRange2
{
    template<int i, typename Tt>
    using spec = Trt<dim, i, Tt>;
};

template<template<int, int, typename>class T, int r1_min, int r2_min, typename... Es, typename I, I... ints1, I... ints2>
TypeData examineTemplateRangeRangeTypes([[maybe_unused]] std::integer_sequence<I, ints1...> r1, [[maybe_unused]] std::integer_sequence<I, ints2...> r2)
{
    static_assert(sizeof...(ints2) > 0, "At least one dimension in r2 is required for examineTemplateRangeRangeTypes().");
    return TypeData({examineTemplateRangeTypes<SpecializeRange2<T, r2_min + ints2>::template spec, r1_min, Es...>(r1)...});
}

std::vector<std::string> postprocessRow(TypeData && td)
{
    std::vector<std::string> row;

    for (const auto v : td.traits_values)
    {
        if (v)
            row.emplace_back("\\cellcolor{" + true_color + "}");
        else
            row.emplace_back("\\cellcolor{" + false_color + "}");
    }


    // strip template parameters in object name and texttt style
    auto x = td.description.find('<') + 1;
    td.description.erase(x, td.description.find('>')-x);
    td.description = "\\texttt{" + td.description + "}";
    row.emplace_back(td.description);

    for (const auto v : td.is_transformable_values)
    {
        if (v)
            row.emplace_back("\\cellcolor{" + true_color + "}");
        else
            row.emplace_back("\\cellcolor{" + false_color + "}");
    }

    return row;
}

std::string generateColumnStyle()
{
    std::string cs;
    for (int i = 0; i < TraitTest::trs_cnt; i++)
        cs.append("c|");
    cs.append("|c|");
    for (int i = 0; i < TransformableTest::tfs_cnt; i++)
        cs.append(("|c"));
    return cs;
}

std::vector<std::string> generateHeading()
{
    std::vector<std::string> heading;

    auto traits_descs = TraitTest::trsNames();
    for (const auto &tr_d : traits_descs)
    {
        auto tmp = tr_d;
        auto x = tmp.rfind(':') + 1;
        tmp = rtl::LaTeX::escapeLaTeXCharacters(tmp.substr(x, tmp.find('<') - x));
        tmp = "\\rotatebox[origin=c]{90}{\\texttt{~" + tmp + "}}";
        heading.emplace_back(tmp);
    }

    heading.emplace_back("\\shortstack{Examined \\\\ templates}");

    auto tf_descs = TransformableTest::tfsNames();
    for (const auto &tf_d : tf_descs)
    {
        auto tmp = tf_d;
        auto x = tmp.rfind(':') + 1;
        tmp = tmp.substr(x, tmp.find("ND<")-x);
        tmp = "\\rotatebox[origin=c]{90}{\\texttt{~" + tmp + "}}";
        heading.emplace_back(tmp);
    }

    return heading;
}

struct ResultTable
{
    rtl::LaTeXTable table;

    template<template<typename> class T>
    void addTTemplate()
    {
        table.addHLine();
        table.addRow(postprocessRow(examineTemplateTypes<T, float, double>()));
    }

    template<template<int, typename> class T>
    void addRTTemplate()
    {
        table.addHLine();
        table.addRow(postprocessRow(examineTemplateRangeTypes<T, 2, float, double>(std::make_integer_sequence<int, 4>())));
    }

    template<template<int, int, typename> class T>
    void addRRTTemplate()
    {
        table.addHLine();
        table.addRow(postprocessRow(examineTemplateRangeRangeTypes<T, 2, 2, float, double>(std::make_integer_sequence<int, 4>(), std::make_integer_sequence<int, 4>())));
    }
};

int main()
{
    // type trait table of the Robotic template library:
    rtl::LaTeXDoc ld("t_type_traits_out", "type_traits_tables");
    ld.setRemoveTmpDir([](const std::string &){ return true; });
    ResultTable rt;

    rt.table.setColumnStyle(generateColumnStyle());
    rt.table.setHeading(generateHeading());
    rt.table.addHLine();
    rt.addRTTemplate<rtl::VectorND>();
    rt.addRTTemplate<rtl::LineSegmentND>();
    rt.addRTTemplate<rtl::BoundingBoxND>();
    rt.table.addHLine();
    rt.addTTemplate<rtl::Polygon2D>();
    rt.addTTemplate<rtl::Polygon3D>();
    rt.addTTemplate<rtl::Frustum3D>();
    rt.table.addHLine();
    rt.addRTTemplate<rtl::TranslationND>();
    rt.addRTTemplate<rtl::RotationND>();
    rt.addRTTemplate<rtl::RigidTfND>();
    rt.table.addHLine();
    rt.addRRTTemplate<rtl::Matrix>();
    rt.addTTemplate<rtl::Quaternion>();

    ld.addTable(rt.table, "Type traits of the Robotic template library, when applied on selected template objects. Type properties are examined in the left part of the table,"
                          "while the applicability of geometrical transformations is summarized to the right. The traits are named in a positive manner, so if e.g. "
                          "an object \\texttt{Obj} has a metric defined, \\texttt{" + rtl::LaTeX::escapeLaTeXCharacters("rtl::has_metric<Obj>::value") + "} is \\colorbox{" +
                          true_color + "}{true}, otherwise it would be \\colorbox{" + false_color + "}{false}.");

    // side test of various usages of the has_random trait:
    auto float_generator = [](){ return 0.0f; };
    std::cout<<"rtl::has_random_v<rtl::Quaternionf>: "<<rtl::has_random_v<rtl::Quaternionf><<std::endl;
    std::cout<<"rtl::has_random_v<rtl::Quaternionf, decltype(float_generator)>: "<<rtl::has_random_v<rtl::Quaternionf, decltype(float_generator)><<std::endl;
    std::cout<<"rtl::has_random_v<rtl::Quaternionf, decltype(float_generator), decltype(float_generator)>: "<<rtl::has_random_v<rtl::Quaternionf, decltype(float_generator), decltype(float_generator)><<std::endl;
    std::cout<<"rtl::has_random_v<rtl::Quaternionf, void>: "<<rtl::has_random_v<rtl::Quaternionf, void><<std::endl;
    std::cout<<"rtl::has_random_v<rtl::Quaternionf, decltype(float_generator), decltype(float_generator), decltype(float_generator)>: "<<rtl::has_random_v<rtl::Quaternionf, decltype(float_generator), decltype(float_generator), decltype(float_generator)><<std::endl;
    return 0;
}