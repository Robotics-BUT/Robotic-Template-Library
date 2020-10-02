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

#ifndef ROBOTICTEMPLATELIBRARY_TFTREENODE_H
#define ROBOTICTEMPLATELIBRARY_TFTREENODE_H

#include <set>
#include <unordered_set>
#include <functional>

namespace rtl
{
    template<typename, typename>
    class TfTree;

    /*!
     * TfTreeNode is a base building block of the TfTree structure for management of transformations and geometrical relationships between coordinate frames. From the user's point of view,
     * TfTreeNode aggregates all content of the node: key, pointer to the parent node, transformation from the parent node to *this, pointers to the child nodes and depth in the tree. The node
     * internally takes care of its connections with other nodes and neither the tree, nor anything other should interfere with this mechanism.
     * @tparam K Key type.
     * @tparam T Transformation type.
     */
    template<typename K, typename T>
    class TfTreeNode
    {
    public:
        friend class TfTree<K, T>;

        using KeyType = K;              //!< Type of the key.
        using TransformationType = T;   //!< Type of the transformation.

        //! Default constructor.
        /*!
         * Default constructor creates a root node with key given by default constructor of \p Key.
         */
        TfTreeNode() : int_depth(0), int_parent(nullptr) {}

        //! Copy constructor.
        /*!
         *
         * @param cp node to be copied.
         */
        TfTreeNode(const TfTreeNode& cp) : int_depth(cp.int_depth), int_key(cp.int_key), tf_from_parent(cp.tf_from_parent), int_parent(cp.int_parent), int_children(cp.int_children)
        {
        }

        //! Move constructor.
        /*!
         *
         * @param mv node to be moved.
         */
        TfTreeNode(TfTreeNode&& mv) noexcept : int_depth(mv.int_depth), int_key(std::move(mv.int_key)), tf_from_parent(mv.tf_from_parent), int_children(std::move(mv.int_children))
        {
            if(int_depth == 0)
            {
                int_parent = this;
            }
            else
            {
                int_parent = mv.int_parent;
                int_parent->int_children.erase(&mv);
                int_parent->int_children.insert(this);
            }
            for (auto c : int_children)
                c->int_parent = this;
        }

        //! Root node constructor.
        /*!
         * Constructs a node with depth 0 and given \p key.
         * @param key key of the root node.
         */
        explicit TfTreeNode(const KeyType &key) : int_key(key), int_parent(this)
        {
            int_depth = 0;
        }

        //! Regular node constructor.
        /*!
         * Constructs a node with all relevant data.
         * @tparam Tf type of the internal transformation.
         * @param key key of the node.
         * @param transformation transformation from the parent node to *this.
         * @param parent pointer to the parent node.
         */
        template<typename Tf>
        TfTreeNode(const KeyType &key, Tf &&transformation, TfTreeNode<K, T>& parent) : int_key(key), tf_from_parent(transformation), int_parent(&parent)
        {
            int_depth = parent.depth() + 1;
            int_parent->int_children.insert(this);
        }

        //! Destructor.
        /*!
         * Automatically erases the node from parent's list of children.
         */
        ~TfTreeNode()
        {
            if (int_depth != 0)
                int_parent->int_children.erase(this);
        }

        TfTreeNode& operator=(const TfTreeNode& cp)=delete;
        TfTreeNode& operator=(TfTreeNode&& mv)=delete;

        //! Key of the node.
        /*!
         *
         * @return reference to node's key.
         */
        [[nodiscard]] const KeyType& key() const
        {
            return int_key;
        }

        //! Parent of the node.
        /*!
         *
         * @return pointer to the parent node.
         */
        [[nodiscard]] TfTreeNode* parent() const
        {
            return int_parent;
        }

        //! Set of children.
        /*!
         *
         * @return reference to the set of pointers to the child nodes.
         */
        [[nodiscard]] const auto& children() const
        {
            return int_children;
        }

        //! Depth in the tree.
        /*!
         *
         * @return depth with respect to root.
         */
        [[nodiscard]] size_t depth() const
        {
            return int_depth;
        }

        //! Transformation from parent to *this.
        /*!
         *
         * @return reference to the transformation.
         */
        [[nodiscard]] const TransformationType& tf() const
        {
            return tf_from_parent;
        }

        //! Transformation from parent to *this.
        /*!
         *
         * @return reference to the transformation.
         */
        [[nodiscard]] TransformationType& tf()
        {
            return tf_from_parent;
        }

    private:
        size_t int_depth;
        KeyType int_key;
        TransformationType tf_from_parent;
        TfTreeNode *int_parent;
        std::set<TfTreeNode*> int_children;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_TFTREENODE_H
