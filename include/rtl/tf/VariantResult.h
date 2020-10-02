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

#ifndef ROBOTICTEMPLATELIBRARY_VARIANTRESULT_H
#define ROBOTICTEMPLATELIBRARY_VARIANTRESULT_H

#include <variant>
#include <type_traits>
#include <experimental/type_traits>

#include "rtl/core/TypeTraits.h"

namespace rtl
{
    template<typename... Tfs>
    class GeneralTf;

    /*!
     * VariantResultType is capable of holding any type listed in the \p Res pack. It has reduced api with respect to std::variant, but can be implicitly converted to any type in \p Res.
     * @tparam Res pack of types.
     */
    template<typename... Res>
    class VariantResultType
    {
    public:
        using VariantType = std::variant<Res...>; //!< Specialization of std::variant holding the internal data.

        static constexpr size_t alternatives_cnt = sizeof...(Res); //!< Number of alternatives held.

        //! Copy constructor.
        /*!
         *
         * @param cp variant result to be copied.
         */
        VariantResultType(const VariantResultType& cp) : result(cp.result)
        {
        }

        //! Move constructor.
        /*!
         *
         * @param mv variant result to be moved.
         */
        VariantResultType(VariantResultType&& mv) noexcept : result(std::move(mv.result))
        {
        }

        //! Constructor from \p Res types.
        /*!
         * Produces compile-time error is \p T is not in \p Res.
         * @tparam T type of the object.
         * @param t object to used in construction.
         */
        template<typename T>
        VariantResultType(const T &t) : result(t)
        {
            static_assert((std::is_same_v<T, Res> || ...), "VariantResult is only constructable from its alternative types.");
        }

        //! Copy-assigment operator.
        /*!
         *
         * @param cp variant result to be copied.
         * @return reference to *this.
         */
        VariantResultType &operator=(const VariantResultType &cp)
        {
            result = cp.result;
            return *this;
        }

        //! Move-assigment operator.
        /*!
         *
         * @param mv variant result to be moved.
         * @return reference to *this.
         */
        VariantResultType &operator=(VariantResultType &&mv) noexcept
        {
            result = std::move(mv.result);
            return *this;
        }

        //! Index of the active type from \p Res.
        /*!
         *
         * @return integral index.
         */
        [[nodiscard]] constexpr size_t index() const noexcept
        {
            return result.index();
        }

        //! Implicit conversion to alternative types from the \p Res pack.
        /*!
         * Produces compile-time error is \p T is not in \p Res and runt-time exception, if wrong alternative from \p Res is requested.
         * @tparam T required type after conversion.
         * @return internal object converted to type \p T.
         */
        template<typename T>
        operator T() const
        {
            static_assert((std::is_same_v<T, Res> || ...), "Conversion is only supported for alternative types of the VariantResult.");
            return std::get<T>(result);
        }

    private:
        VariantType result;
    };

    template<typename T, typename... Ts>
    struct Unique
    {
        using type = T;
    };

    template<typename... Us, typename T, typename... Ts>
    struct Unique<VariantResultType<Us...>, T, Ts...>
            : std::conditional_t<((std::is_same_v<T, Us> || ...) || std::is_same_v<T, void>), Unique<VariantResultType<Us...>, Ts...>, Unique<VariantResultType<Us..., T>, Ts...>>
    {
    };

    template<typename VRTKnown, typename VRTNew, typename...>
    struct Expansion
    {
        using type = VRTNew;
    };

    template<typename... VrsKnown, typename... VrsNew, typename T, typename... Tfs>
    struct Expansion<VariantResultType<VrsKnown...>, VariantResultType<VrsNew...>, T, Tfs...>
            : Expansion<VariantResultType<VrsKnown...>, typename Unique<VariantResultType<VrsKnown...>, VrsNew..., decltype(std::declval<VrsKnown>().transformed(std::declval<T>()))...>::type, Tfs...>
    {
    };

    template<typename VR1, typename...>
    struct Recursion
    {
        using type = VR1;
    };

    template <typename... Vrs1, typename... Vrs2, typename... Tfs>
    struct Recursion<VariantResultType<Vrs1...>, VariantResultType<Vrs2...>, Tfs...>
            : std::conditional_t<(std::is_same_v<VariantResultType<Vrs1...>, VariantResultType<Vrs2...>>),
            Recursion<VariantResultType<Vrs1...>>, Recursion<VariantResultType<Vrs2...>, typename Expansion<VariantResultType<Vrs2...>, VariantResultType<Vrs2...>, Tfs...>::type>>
    {
    };

    template<typename T>
    struct StripIfUnique
    {
        using type = T;
    };

    template<typename T, typename... Ts>
    struct StripIfUnique<VariantResultType<T, Ts...>>
    {
        using type = typename std::conditional<sizeof...(Ts) == 0, T, VariantResultType<T, Ts...>>::type;
    };

    //! Variant result covering all possible output types resulting from transformation of type \p Obj by transformations of types \p Tfs.
    template<typename Obj, typename... Tfs>
    using VariantResultOTs = typename Unique<VariantResultType<>, typename std::experimental::detected_or<void, TransformedResult, Obj, Tfs>::type ...>::type;

    //! Variant result covering all possible output types resulting from transformation of type \p Obj by transformations of types \p Tfs. If the variant holds only one type, it is used directly.
    template <typename Obj, typename... Tfs>
    using VariantResultOTsStripped = typename StripIfUnique<VariantResultOTs<Obj, Tfs...>>::type;

    //! Variant result covering all possible output types resulting from transformation of objects \p Objs by transformation of types \p Tf.
    template <typename Tf, typename... Objs>
    using VariantResultTOs = typename Unique<VariantResultType<>, typename std::experimental::detected_or<void, TransformedResult, Objs, Tf>::type ...>::type;

    //! Variant result covering all possible output types resulting from transformation of objects \p Objs by transformation of types \p Tf. If the variant holds only one type, it is used directly.
    template <typename Tf, typename... Objs>
    using VariantResultTOsStripped = typename StripIfUnique<VariantResultTOs<Tf, Objs...>>::type;


    template<typename Obj, typename... Tfs>
    struct is_transformable<Obj, GeneralTf<Tfs...>>
    {
        using type = VariantResultOTsStripped<Obj, Tfs...>;
        static constexpr bool value = VariantResultOTs<Obj, Tfs...>::alternatives_cnt > 0;
    };


    template <typename Obj, typename... Tfs>
    struct VariantResultRecursiveType
    {
        using type = typename StripIfUnique<typename Recursion<VariantResultOTs<Obj, Tfs...>, Expansion<VariantResultOTs<Obj, Tfs...>, VariantResultOTs<Obj, Tfs...>, Tfs...>>::type>::type;
    };

    template <typename Obj, typename... Tfs>
    struct VariantResultRecursiveType<Obj, GeneralTf<Tfs...>>
    {
        using type = typename Recursion<VariantResultOTs<Obj, Tfs...>, Expansion<VariantResultOTs<Obj, Tfs...>, VariantResultOTs<Obj, Tfs...>, Tfs...>>::type;
    };

    /*!
     * Variant result covering all possible output types resulting from transformation of type \p Obj by transformations of types \p Tfs. Output types are recursively transformed again
     * and if a new type appears, it is added to to the pack of possible output types. This process is repeated until no new output types are obtained resulting in a closed set of possible
     * outputs regardless of order or number applications of transformations from \p Tfs.
     */
    template <typename Obj, typename... Tfs>
    using VariantResultRecursive = typename VariantResultRecursiveType<Obj, Tfs...>::type;

    /*!
     * Variant result covering all possible output types resulting from transformation of type \p Obj by transformations of types \p Tfs. Output types are recursively transformed again
     * and if a new type appears, it is added to to the pack of possible output types. This process is repeated until no new output types are obtained resulting in a closed set of possible
     * outputs regardless of order or number applications of transformations from \p Tfs. If the variant holds only one type, it is used directly.
     */
    template <typename Obj, typename... Tfs>
    using VariantResultRecursiveStripped = typename StripIfUnique<typename VariantResultRecursiveType<Obj, Tfs...>::type>::type;

}

#endif //ROBOTICTEMPLATELIBRARY_VARIANTRESULT_H
