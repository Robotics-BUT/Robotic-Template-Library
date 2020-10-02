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

#ifndef ROBOTICTEMPLATELIBRARY_TYPETRAITS_H
#define ROBOTICTEMPLATELIBRARY_TYPETRAITS_H

namespace rtl
{
    template<typename T>
    using ElementTypeType = typename T::ElementType;

    //! Tests whether type \p T has an ElementType defined.
    /*!
     *
     * @tparam T type to be tested.
     */
    template<typename T>
    struct has_element_type
    {
        //! T::ElementType if available, std::experimental::nonesuch otherwise.
        using type = typename std::experimental::is_detected<ElementTypeType, T>::type;
        //! True if T::ElementType is defined, false otherwise.
        static constexpr bool value = std::experimental::is_detected<ElementTypeType, T>::value;
    };

    //! Type of the has_element_type type trait.
    template<typename T>
    using has_element_type_t = typename has_element_type<T>::type;

    //! Value of the has_element_type trait.
    /*!
     *
     * @tparam T type to be tested.
     */
    template<typename T>
    constexpr bool has_element_type_v = has_element_type<T>::value;


    template<typename... Res>
    class VariantResultType;

    //! Tests if a type \p T is a specialization of rtl::VariantResultType template.
    /*!
     *
     * @tparam T type to be examined.
     */
    template<typename T>
    struct is_variant_result
    {
        static constexpr bool value = false; //!< true if \p T is a specialization of rtl::VariantResultType template, false otherwise.
    };

    template<typename... Ts>
    struct is_variant_result<VariantResultType < Ts...>>
    {
        static constexpr bool value = true;
    };

    //! Value of the is_variant_result type trait.
    /*!
     * True if \p T is a specialization of rtl::VariantResultType template, false otherwise.
     * @tparam T type to be examined.
     */
    template<typename T>
    constexpr bool is_variant_result_v = is_variant_result<T>::value;


    template<typename O, typename T>
    using TransformedResult = decltype(std::declval<O &>().transformed(std::declval<T &>()));

    //! Tests if an object of type \p Obj can be transformed by a transformation of type \p Tf.
    /*!
     * For transformation of an object by rtl::GeneralTf, the value is true, if at least one of the alternative transformations gives valid result (others might be invalid).
     * @tparam Obj type of the object to be transformed.
     * @tparam Tf type of the transformation applied.
     */
    template<typename Obj, typename Tf>
    struct is_transformable
    {
        //! Type of the result of transformation of \p Obj by \p Tf if valid, std::experimental::nonesuch otherwise.
        using type = typename std::experimental::is_detected<TransformedResult, Obj, Tf>::type;
        //! True if \p Obj can be transformed by \p Tf, false otherwise.
        static constexpr bool value = std::experimental::is_detected<TransformedResult, Obj, Tf>::value;
    };

    //! Type of the is_transformable type trait.
    template<typename Obj, typename Tf>
    using is_transformable_t = typename is_transformable<Obj, Tf>::type;

    //! Value of the is_transformable type trait.
    /*!
     *
     * @tparam Obj type of the object to be transformed.
     * @tparam Tf type of the transformation applied.
     */
    template<typename Obj, typename Tf>
    constexpr bool is_transformable_v = is_transformable<Obj, Tf>::value;


    template<typename T>
    using InvertedResult = decltype(std::declval<T &>().inverted());

    //! Type trait testing if type \p T can be inverted.
    /*!
     *
     * @tparam T type to be tested.
     */
    template<typename T>
    struct is_invertible
    {
        //! Type of the inverted type \p T, std::experimental::nonesuch otherwise.
        using type = typename std::experimental::is_detected<InvertedResult, T>::type;
        //! True if \p T can be inverted, false otherwise.
        static constexpr bool value = std::experimental::is_detected<InvertedResult, T>::value;
    };

    //! Type of the is_invertible type trait.
    template<typename T>
    using is_invertible_t = typename is_invertible<T>::type;

    //! Value of the is_invertible trait.
    /*!
     *
     * @tparam T type tested for identity() presence.
     */
    template<typename T>
    constexpr bool is_invertible_v = is_invertible<T>::value;


    template<typename T>
    using IdentityResult = decltype(T::identity());

    //! Tests whether type \p T has a static identity method.
    /*!
     *
     * @tparam T type of the tested object.
     */
    template<typename T>
    struct has_identity
    {
        //! Type of the result of T::identity() invocation if available, std::experimental::nonesuch otherwise.
        using type = typename std::experimental::is_detected<IdentityResult, T>::type;
        //! True if \p T has a static identity() method, false otherwise.
        static constexpr bool value = std::experimental::is_detected<IdentityResult, T>::value;
    };

    //! Type of the has_identity type trait.
    template<typename T>
    using has_identity_t = typename has_identity<T>::type;

    //! Value of the has_identity trait.
    /*!
     *
     * @tparam T type tested for identity() presence.
     */
    template<typename T>
    constexpr bool has_identity_v = has_identity<T>::value;


    template<typename T>
    using NaNResult = decltype(T::nan());

    //! Tests whether type \p T has a static nan() method.
    /*!
     *
     * @tparam T type of the tested object.
     */
    template<typename T>
    struct has_nan
    {
        //! Type of the result of T::nan() invocation if available, std::experimental::nonesuch otherwise.
        using type = typename std::experimental::is_detected<NaNResult, T>::type;
        //! True if \p T has a static nan() method, false otherwise.
        static constexpr bool value = std::experimental::is_detected<NaNResult, T>::value;
    };

    //! Type of the has_nan type trait.
    template<typename T>
    using has_nan_t = typename has_nan<T>::type;

    //! Value of the has_nan trait.
    /*!
     *
     * @tparam T type tested for nan() presence.
     */
    template<typename T>
    constexpr bool has_nan_v = has_nan<T>::value;



    template<typename Obj, typename... Gs>
    using RandomResult = decltype(Obj::random(std::declval<Gs &>()...));

    template<typename... Ts>
    struct HasRandomImpl
    {
        static_assert("has_random trait needs at least one template argument specifying the tested object.");
    };

    template<typename Obj>
    struct HasRandomImpl<Obj>
    {
        static_assert(has_element_type_v<Obj>, "has_random trait without any generator specified requires the tested object to implement the ElementType.");
        using E = typename Obj::ElementType;
        struct G
        {
            E operator()()
            {
                return E();
            }
        };

        using type = typename std::experimental::is_detected<RandomResult, Obj, G>::type;
        static constexpr bool value = std::experimental::is_detected<RandomResult, Obj, G>::value;
    };

    template<typename Obj, typename G, typename... Gs>
    struct HasRandomImpl<Obj, G, Gs...>
    {
        using type = typename std::experimental::is_detected<RandomResult, Obj, G, Gs...>::type;
        static constexpr bool value = std::experimental::is_detected<RandomResult, Obj, G, Gs...>::value;
    };


    //! Tests whether type \p T has a static random method.
    /*!
     * random() methods of various templates in RTL have not the same arguments in all cases. In principle there needs to be at least one random element generating object,
     * but more are possible as well. The has_random trait takes the tested type \p Obj as the first template parameter, the rest is not mandatory. If no type of the random
     * generator is provided, the has_random tries to extract ElementType of \p Obj, if successful, makes a generator of this type \a G and checks for Obj::random(G&) static
     * method. If \p RndGen pack contains any types, Obj::random(std::declval<RndGen &>()...) is examined.
     *
     * @tparam Obj type of the tested object.
     * @tparam RndGen pack of types of random element generators.
     */
    template<typename Obj, typename... RndGen>
    struct has_random
    {
        //! Type of the result of Obj::random(RndGen &) invocation if available, std::experimental::nonesuch otherwise.
        using type = typename HasRandomImpl<Obj, RndGen...>::type;
        //! True if \p T has a static random(RndGen &) method, false otherwise.
        static constexpr bool value = HasRandomImpl<Obj, RndGen...>::value;
    };

    //! Type of the has_random type trait.
    template<typename Obj, typename... RndGen>
    using has_random_t = typename has_random<Obj, RndGen...>::type;

    //! Value of the has_random trait.
    /*!
     *
     * @tparam Obj type tested for random(RndGen &) presence.
     * @tparam RndGen pack of types of random element generators.
     */
    template<typename Obj, typename... RndGen>
    constexpr bool has_random_v = has_random<Obj, RndGen...>::value;


    template<typename T>
    using DistanceResult = decltype(T::distance(std::declval<T &>(), std::declval<T &>()));

    //! Tests whether type \p T has metric (aka static distance function) defined.
    /*!
     *
     * @tparam T type to be tested.
     */
    template<typename T>
    struct has_metric
    {
        //! Type of the result of T::distance(T&, T&) invocation if available, std::experimental::nonesuch otherwise.
        using type = typename std::experimental::is_detected<DistanceResult, T>::type;
        //! True if \p T has a static distance function defined, false otherwise.
        static constexpr bool value = std::experimental::is_detected<DistanceResult, T>::value;
    };

    //! Type of the has_metric type trait.
    template<typename T>
    using has_metric_t = typename has_metric<T>::type;

    //! Value of the has_metric trait.
    /*!
     *
     * @tparam T type to be tested.
     */
    template<typename T>
    constexpr bool has_metric_v = has_metric<T>::value;


    template<typename T>
    using DimensionalityResult = decltype(T::dimensionality());

    //! Tests whether type \p T resides in a space of some dimension.
    /*!
     *
     * @tparam T type to be tested.
     */
    template<typename T>
    struct is_dimensional
    {
        //! Type of the result of T::dimensionality() invocation if available, std::experimental::nonesuch otherwise.
        using type = typename std::experimental::is_detected<DimensionalityResult, T>::type;
        //! True if \p T has a static dimensionality function defined, false otherwise.
        static constexpr bool value = std::experimental::is_detected<DimensionalityResult, T>::value;
    };

    //! Type of the is_dimensional type trait.
    template<typename T>
    using is_dimensional_t = typename is_dimensional<T>::type;

    //! Value of the is_dimensional trait.
    /*!
     *
     * @tparam T type to be tested.
     */
    template<typename T>
    constexpr bool is_dimensional_v = is_dimensional<T>::value;

}

#endif //ROBOTICTEMPLATELIBRARY_TYPETRAITS_H
