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

#ifndef ROBOTICTEMPLATELIBRARY_GENERALTF_H
#define ROBOTICTEMPLATELIBRARY_GENERALTF_H

#include <variant>
#include <type_traits>
#include "rtl/tf/VariantResult.h"

namespace rtl
{
    /*!
     * GeneralTf is a class capable of holding any transformation listed in the \p Tfs pack similar to std::variant. General transformation supports much of the interface of other regular
     * transformations, such as transform(), transformed(), operator() and other methods. Practical usage quickly shows, that for some combination of transformed objects and types
     * in \p Tfs, there might be different outputs of the same function depending on active alternative from the \p Tfs pack, or the transformation might not be defined at all.
     * General transformation offers much grater flexibility that std::variant in such cases. Compile time error is generated if none of the alternatives can be used in given situation and
     * exceptions are thrown, if bad alternative is used at runtime. This ensures type-safety and selects an optimal variant (compile-time over run-time check). Uncertainty of
     * the return type is solved via the VariantResultType class described elsewhere. If only one output type is possible, VariantResultType is not used at all.
     * @tparam Tfs pack of types of all possibly stored transformations.
     */
    template<typename... Tfs>
    class GeneralTf
    {
        static_assert(sizeof...(Tfs) > 0, "rtl::GeneralTf must be instantiated for at least one transformation type.");
    public:
        template<size_t I>
        using AlternativeType = std::variant_alternative_t<I, std::variant<Tfs...>>;

        //! Default constructor.
        /*!
         * General transformation holds uninitialized, may be even invalid transformation after default construction.
         */
        GeneralTf() = default;

        //! Copy constructor.
        /*!
         *
         * @param cp GeneralTF to be copied.
         */
        GeneralTf(const GeneralTf<Tfs...> &cp) : int_tf(cp.int_tf) {}

        //! Move constructor.
        /*!
         *
         * @param mv GeneralTF to be moved.
         */
        GeneralTf(GeneralTf<Tfs...> &&mv) noexcept : int_tf(std::move(mv.int_tf)) {}

        //! Template constructor for construction from supported transformation types.
        /*!
         *
         * @tparam T type of the transformation (must be in the \p Tfs pack).
         * @param tf the transformation.
         */
        template<typename T>
        GeneralTf(const T &tf) : int_tf(tf) {}

        //! Destructor.
        ~GeneralTf() = default;

        //! Copy-assignment operator.
        /*!
         *
         * @param cp general transformation to be copied.
         * @return reference to *this.
         */
        GeneralTf<Tfs...>& operator=(const GeneralTf<Tfs...> &cp)
        {
            int_tf = cp.int_tf;
            return *this;
        }

        //! Move-assigment operator.
        /*!
         *
         * @param mv general transformation to be moved.
         * @return reference to *this.
         */
        GeneralTf<Tfs...>& operator=(GeneralTf<Tfs...> &&mv) noexcept
        {
            int_tf = std::move(mv.int_tf);
            return *this;
        }

        //! Templated assigment operator for supported transformation types.
        /*!
         *
         * @tparam T type of the transformation (must be in the \p Tfs pack).
         * @param mv assigned transformation.
         * @return reference to *this.
         */
        template<typename T>
        GeneralTf<Tfs...>& operator=(T &&mv) noexcept
        {
            int_tf = mv;
            return *this;
        }

        //! Index of the active type from the \p Tfs pack.
        /*!
         *
         * @return index of the active alternative.
         */
        [[nodiscard]] constexpr size_t index() const noexcept
        {
            return int_tf.index();
        }

        //! Visit mechanism for contained transformations.
        /*!
         * Functionally equivalent to std::visit of std::variant.
         * @tparam Visitor visitor type.
         * @param vis visitor object.
         * @return output of the visitor.
         */
        template<typename Visitor>
        auto visit(Visitor &&vis)
        {
            return std::visit(vis, int_tf);
        }

        //! Visit mechanism for contained transformations.
        /*!
         * Functionally equivalent to std::visit of std::variant.
         * @tparam Visitor visitor type.
         * @param vis visitor object.
         * @return output of the visitor.
         */
        template<typename Visitor>
        auto visit(Visitor &&vis) const
        {
            return std::visit(vis, int_tf);
        }

        //! In-place inversion of the contained transformation.
        void invert()
        {
            std::visit([this](auto&& tr){ int_tf = tr.inverted();}, int_tf);
        }

        //! Returns inverted variant of *this.
        /*!
         *
         * @return inverted variant of *this.
         */
        GeneralTf inverted() const
        {
            GeneralTf inv(*this);
            inv.invert();
            return inv;
        }

        //! Functor call invoking the transformed(*this) method of given argument.
        /*!
         * Note that the transformed object \p obj does not have to implement the transformed() method for general transformation input, only the type of active alternative in GeneralTf
         * needs to be supported.
         * @tparam T type of the transformed object.
         * @param obj the transformed object.
         * @return the result of the transformation.
         */
        template<typename T>
        auto operator()(const T &obj) const
        {
            using OutputType = VariantResultOTsStripped<T, Tfs...>;
            typedef OutputType(*FuncPtr)(const T&, const std::variant<Tfs...>&);
            static constexpr FuncPtr tf_functions[] = {(&GeneralTf::validlyTransformObjByGtf<OutputType, Tfs, T>)...};
            return tf_functions[int_tf.index()](obj, int_tf);
        }

        //! Functor call transformation of another general transformation object.
        /*!
         *
         * @tparam Tfs2 possible alternatives of the transformed object.
         * @param gtf general transformation to be transformed.
         * @return the result of the transformation.
         */
        template<typename... Tfs2>
        GeneralTf operator()(const GeneralTf<Tfs2...> &gtf) const
        {
            return gtf.transformed(*this);
        }

        //! Templated getter of the contained transformation.
        /*!
         * Throws std::bad_variant_access if wrong alternative was chosen.
         * @tparam T Type to be extracted.
         * @return extracted transformation of the type \p T.
         */
        template<typename T>
        T tf() const
        {
            return std::get<T>(int_tf);
        }

        //! Returns general transformation transformed by \p tr.
        /*!
         *
         * @tparam T type of the transformation.
         * @param tr the transformation applied on *this.
         * @return new general transformation.
         */
        template<typename T>
        GeneralTf transformed(const T &tr) const
        {
            GeneralTf ret(*this);
            ret.transform(tr);
            return ret;
        }

        //! In-place augments *this to perform first transformation by *this and than transformation by \p tr.
        /*!
         *
         * @tparam T type of the transformation.
         * @param tf the transformation applied on *this.
         */
        template<typename T>
        void transform(const T &tf)
        {
            typedef void(*FuncPtr)(GeneralTf&, const T&);
            static constexpr FuncPtr tf_functions[] = {(&GeneralTf::validlyTransformGtfByTf<T, Tfs>)...};
            tf_functions[int_tf.index()](*this, tf);
        }

        //! In-place augments *this to perform first transformation by *this and than transformation by \p tr.
        /*!
         *
         * @tparam Tfs2 pack of types possibly hold by the general transformation applied on *this.
         * @param gtf the general transformation applied on *this.
         */
        template<typename... Tfs2>
        void transform(const GeneralTf<Tfs2...> &gtf)
        {
            typedef void(*FuncPtr)(GeneralTf&, const GeneralTf<Tfs2...>&);
            static constexpr FuncPtr tf_functions[] = {(&GeneralTf::validlyTransformGtfByGtf<GeneralTf, Tfs2>)...};
            tf_functions[gtf.index()](*this, gtf);
        }

        //! Returns a new general transformation, which, when applied, leaves the transformed object unchanged.
        /*!
         * The first type in the \p Tfs pack implementing the identity() static function is used as an active type holding the identity. If no such type exists, compile-time error appears.
         * @return a new general transformation.
         */
        static GeneralTf identity()
        {
            return firstValidIdentity<0>();
        }

        //! Returns a new general transformation initialized by the user-supplied random generator.
        /*!
         * The first type in the \p Tfs pack implementing the random() static function is used as an active type holding the random transformation. If no such type exists,
         * compile-time error appears.
         * @tparam ElRndSrc type of the random generating object.
         * @param el_rnd_gen callable object returning ElementType on invocation.
         * @return a new general transformation initialized by \p el_rnd_gen.
         */
        template<class ElRndSrc>
        static GeneralTf random(const ElRndSrc &el_rnd_gen)
        {
            return firstValidRandom<0>(el_rnd_gen);
        }

        //! Returns a new general transformation initialized by the user-supplied random generator and selector of an alternative from the \p Tfs pack.
        /*!
         * \p tf_rnd_gen is required to return an integral type in range [0, sizeof...(Tfs)]. Selecting a transformation without a static random() function causes run-time exception.
         * Elements produced by the random element generator \p el_rnd_gen are always statically casted to the ElementType of the selected alternative from the \p Tfs pack.
         * @tparam ElRndSrc type of the random element generating object.
         * @tparam TfRndSrc type of the random alternative generating object.
         * @param el_rnd_gen callable object returning ElementType on invocation.
         * @param tf_rnd_gen callable object returning integral index in range [0, sizeof...(Tfs)] on invocation.
         * @return a new random general transformation.
         */
        template<class ElRndSrc, class TfRndSrc>
        static GeneralTf random(const ElRndSrc &el_rnd_gen, const TfRndSrc &tf_rnd_gen)
        {
            typedef GeneralTf(*RndGenPtr)(const ElRndSrc &);
            static RndGenPtr tf_rnd_callbacks[] = {(&validRandomGtf<Tfs, ElRndSrc>)...};
            return tf_rnd_callbacks[tf_rnd_gen()](el_rnd_gen);
        }

        //! Implicit conversion to alternative types from the \p Tfs pack.
        /*!
         * Produces compile-time error is \p T is not in \p Tfs and runt-time exception, if wrong alternative from \p Tfs is requested.
         * @tparam T required type after conversion.
         * @return internal transformation converted to type \p T.
         */
        template<typename T>
        operator T()
        {
            static_assert((std::is_same_v<T, Tfs> || ...), "Implicit conversion is only supported for alternative types given in the GeneralTf parameter pack.");
            return std::get<T>(int_tf);
        }

    private:
        //! Function for generation of custom v-table of the operator().
        /*!
         * @tparam Output output type of the operator() invocation.
         * @tparam Alternative alternative from \p Tfs pack of types to be used.
         * @tparam Object type of the transformed object.
         * @param obj the transformed object.
         * @param var_tf internal variant of the GeneralTf holding the transformation to be applied.
         * @return a transformed object.
         */
        template<typename Output, typename Alternative, typename Object>
        static Output validlyTransformObjByGtf(const Object& obj, const std::variant<Tfs...>& var_tf)
        {
            if constexpr (is_transformable_v<Object, Alternative>)
                return Output(std::get<Alternative>(var_tf)(obj));
            else
                throw std::bad_variant_access();
        }

        //! Function for generation of custom v-table of the transform(const T&) function.
        /*!
         * Transforms \p gtf in-place.
         * @tparam Tf type of the transformation to be applied.
         * @tparam Alternative alternative from \p Tfs pack of types to be used.
         * @param gtf General transformation to be transformed by \p tf.
         * @param tf transformation applied on \p gtf.
         */
        template<typename Tf, typename Alternative>
        static void validlyTransformGtfByTf(GeneralTf& gtf, const Tf& tf)
        {
            if constexpr (is_transformable_v<Alternative, Tf>)
                gtf.int_tf = tf(std::get<Alternative>(gtf.int_tf));
            else
                throw std::bad_variant_access();
        }

        //! Function for generation of custom v-table of the transform(const Gtf2&) function.
        /*!
         * Selects \p Alternative from \p gtf2 and applies it on \p gtf_obj using its transform(const T&) method.
         * @tparam Gtf2 type of the general transformation applied on \p gtf_obj.
         * @tparam Alternative type selected from \p Gtf2 alternatives.
         * @param gtf_obj general transformation to be transformed.
         * @param gtf2 general transformation to be applied on \p gtf_obj.
         */
        template<typename Gtf2, typename Alternative>
        static void validlyTransformGtfByGtf(GeneralTf& gtf_obj, const Gtf2& gtf2)
        {
            gtf_obj.transform(std::get<Alternative>(gtf2.int_tf));
        }

        //! Returns a new general transformation holding \p id -th alternative representing the identity transformation.
        /*!
         * If the \p id -th type does not have the identity() static function implemented, the next type is examined via recursion.
         * @tparam id index of the alternative.
         * @return general transformation leaving the transformed object unchanged.
         */
        template<int id>
        static GeneralTf firstValidIdentity()
        {
            static_assert(id < sizeof...(Tfs), "No alternative type provides static identity() function.");
            using Tf = typename std::tuple_element<id, std::tuple<Tfs...>>::type;
            if constexpr (has_identity_v<Tf>)
                return GeneralTf(Tf::identity());
            else
                return firstValidIdentity<id + 1>();
        }

        //! Returns a new general transformation holding \p id -th alternative representing a random transformation.
        /*!
         * If the \p id -th type does not have the random(ElRndSrc &) static function implemented, the next type is examined via recursion.
         * @tparam id index of the alternative.
         * @tparam ElRndSrc type of the random element generator.
         * @param el_rnd_gen the random element generator.
         * @return random general transformation.
         */
        template<int id, typename ElRndSrc>
        static GeneralTf firstValidRandom(const ElRndSrc &el_rnd_gen)
        {
            static_assert(id < sizeof...(Tfs), "No alternative type provides static random() function.");
            using Tf = typename std::tuple_element<id, std::tuple<Tfs...>>::type;
            if constexpr (has_random_v<Tf, ElRndSrc>)
                return GeneralTf(Tf::random(CastingElRndGen<typename Tf::ElementType, ElRndSrc>(el_rnd_gen)));
            else
                return firstValidRandom<id + 1>(el_rnd_gen);
        }

        //! Function for generation of custom v-table of the random() function.
        /*!
         * @tparam Tf type of the transformation.
         * @tparam ElRndSrc type of the random element generating object.
         * @param el_rnd_gen callable object returning ElementType on invocation.
         * @return random transformation of type \p T.
         */
        template<typename Tf, class ElRndSrc>
        static GeneralTf validRandomGtf(const ElRndSrc &el_rnd_gen)
        {
            if constexpr (has_random_v<Tf, ElRndSrc>)
                return GeneralTf(Tf::random(CastingElRndGen<typename Tf::ElementType, ElRndSrc>(el_rnd_gen)));
            else
                throw std::bad_variant_access();
        }

        //! Proxy-class casting output of given random element generator to type \p C.
        /*!
         *
         * @tparam C type f the generated elements after cast.
         * @tparam ElGen type of the random element generator.
         */
        template<typename C, typename ElGen>
        struct CastingElRndGen
        {
            CastingElRndGen(const ElGen &eg) : el_gen(eg) {}
            C operator()() const { return static_cast<C>(el_gen()); }
            const ElGen &el_gen;
        };

        std::variant<Tfs...> int_tf;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_GENERALTF_H
