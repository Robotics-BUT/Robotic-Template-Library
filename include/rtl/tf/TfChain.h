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

#ifndef ROBOTICTEMPLATELIBRARY_TFCHAIN_H
#define ROBOTICTEMPLATELIBRARY_TFCHAIN_H

#include <list>
#include <type_traits>


namespace rtl
{
    /*!
     * TfChain is a container class for storing a list of transformations. Works with GeneralTf, so multiple types of regular transformations can be kept in one TfChain and
     * can be applied via operator() on an object, consecutively applying all transformations in the chain. Now it is only used as a return type of queries on transformations
     * between nodes on the TfTree class.
     * @tparam T type of the transformations stored.
     */
    template<typename T>
    class TfChain
    {
    public:
        using TransformationType = T;

        //! Default constructor is deleted.
        TfChain()=default;

        //! Copy constructor.
        /*!
         *
         * @param cp transformation chain to be copied.
         */
        TfChain(const TfChain& cp) : tfs_list(cp.tfs_list) {}

        //! Move constructor.
        /*!
         *
         * @param mv transformation chain to be moved.
         */
        TfChain(TfChain&& mv) noexcept : tfs_list(std::move(mv.tfs_list)) {}

        //! Constructor from a std::list of transformations.
        /*!
         *
         * @param list transformations to be stored in the transformation chain.
         */
        explicit TfChain(const std::list<TransformationType> &list) : tfs_list(list) {}

        //! Copy-assignment operator.
        /*!
         *
         * @param cp transformation chain to be copied.
         * @return reference to *this.
         */
        TfChain& operator=(const TfChain& cp)
        {
            tfs_list = cp.tfs_list;
            return *this;
        }

        //! Move-assigment operator.
        /*!
         *
         * @param mv transformation chain to be moved.
         * @return reference to *this.
         */
        TfChain& operator=(TfChain&& mv) noexcept
        {
            tfs_list = std::move(mv.tfs_list);
            return *this;
        }

        //! Destructor.
        ~TfChain()=default;

        //! Functor call consecutively applying all transformations in the chain on \p obj.
        /*!
         * Produces compile-error of if TransformationType is not compatible with \p Object and run-time exceptions, if the chain empty or TransformationType is GeneralTf
         * with wrong alternative.
         * @tparam Object Type of the object to be transformed.
         * @param obj the object to be transformed.
         * @return a new transformed object.
         */
        template<typename Object>
        auto operator()(const Object &obj) const
        {
            using OutputType = VariantResultRecursiveStripped<Object, T>;

            if (tfs_list.empty())
                throw std::out_of_range("Transforming by an empty TfChain.");

            if constexpr (is_variant_result_v<OutputType>)
            {
                static constexpr auto tf_functions = ValidTransforms<OutputType>::array;
                OutputType ret(tfs_list.front()(obj));
                for(auto it = std::next(tfs_list.begin()); it != tfs_list.end(); it++)
                    ret = tf_functions[ret.index()](ret, *it);
                return ret;
            }
            else
            {
                OutputType ret(tfs_list.front()(obj));
                for(auto it = std::next(tfs_list.begin()); it != tfs_list.end(); it++)
                    ret = (*it)(ret);
                return ret;
            }
        }

        //! Reference access to the internal list of transformations.
        /*!
         *
         * @return reference the list of contained transformations.
         */
        [[nodiscard]] const std::list<TransformationType>& list() const
        {
            return tfs_list;
        }

        //! Tries to squash adjacent transformations together producing a single transformation representing the same transformation as the whole chain.
        /*!
         * If TransformationType transformed by another transformation type may throw, the squash() method may throw as well. This can happen when using GeneralTf with some incompatible
         * alternatives as the TransformationType.
         * @return squashed chain.
         */
        TransformationType squash() const
        {
            auto aggregation  = TransformationType::identity();
            for (const auto& t : tfs_list)
                aggregation.transform(t);
            return aggregation;
        }

    private:
        //! Function for generation of custom v-table of the operator().
        /*!
         *
         * @tparam Output type of the output of the transformation.
         * @tparam Alternative alternative from GeneralTf to be used.
         * @param obj object to be transformed.
         * @param var_tf transformation to be applied.
         * @return a new transformed object.
         */
        template<typename Output, typename Alternative>
        static Output validlyTransform(const Output& obj, const TransformationType& var_tf)
        {
            if constexpr (is_transformable_v<Alternative, TransformationType>)
                return Output(var_tf(static_cast<Alternative>(obj)));
            else
                throw std::bad_variant_access();
        }

        template<typename Output>
        struct ValidTransforms {};

        //! Type holding all valid transformations of a VariantResultType by TransformationType.
        /*!
         * Contains static array of pointers to valid transformations of the VariantResultType<Objs...>.
         * @tparam Objs pack of types possibly stored in the VariantResultType.
         */
        template<typename... Objs>
        struct ValidTransforms<VariantResultType < Objs...>>
        {
            using Output = VariantResultType<Objs...>;
            using FuncPtr = Output(*)(const Output &, const TransformationType &);
            static constexpr FuncPtr array[] = {(&TfChain::validlyTransform<Output, Objs>)...};
        };

        std::list<TransformationType> tfs_list;
    };
}


#endif //ROBOTICTEMPLATELIBRARY_TFCHAIN_H
