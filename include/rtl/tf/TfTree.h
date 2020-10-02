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

#ifndef ROBOTICTEMPLATELIBRARY_TFTREE_H
#define ROBOTICTEMPLATELIBRARY_TFTREE_H

#include <map>
#include <list>

#include "TfTreeNode.h"
#include "GeneralTf.h"
#include "VariantResult.h"
#include "TfChain.h"

namespace rtl
{
    /*!
     * TfTree is a tree structure for organization of the geometric relationships between different coordinate frames (or poses, we use these as equivalents). Each pose corresponds to a node
     * in the tree and is uniquely identified by a key. Transformations between adjacent coordinate frames correspond to the edges of the tree graph. Tree-like structure forbids cycles in the
     * resulting graph, therefore between any two nodes, there is exactly one unique chain of transformations, which eliminates potential inconsistencies. The tree cannot be crated without
     * the root node.
     * @tparam K Key type.
     * @tparam T Transformation type.
     */
    template<typename K, typename T>
    class TfTree
    {
    public:
        typedef K KeyType;              //!< Type of the keys.
        typedef T TransformationType;   //!< Type of the transformations between nodes.
        typedef TfTreeNode<KeyType, TransformationType> NodeType;   //!< Type of the nodes in the tree.

        TfTree() = delete;

        //! Base TfTree constructor.
        /*!
         * TfTree cannot be constructed without root, therefore the implicit constructor is disabled anf the key of the root node has to be passed.
         * @param root_key key of the root node.
         */
        explicit TfTree(const KeyType root_key)
        {
            root_node_key = root_key;
            nodes.emplace(root_key, NodeType(root_key));
        }

        //! Copy constructor.
        /*!
         * Creates a deep copy of the tree.
         * @param cp tree to by copied.
         */
        TfTree(const TfTree &cp) : nodes(cp.nodes), root_node_key(cp.root_node_key) {}

        //! Move constructor.
        /*!
         * Moves content of \p mv to a new tree. The root node is than copied back to keep \p mv valid.
         * @param mv tree to be moved from.
         */
        TfTree(TfTree &&mv) noexcept: nodes(mv.nodes), root_node_key(mv.root())
        {
            mv.insertRoot(root_node_key);
        }

        //! Destructor.
        ~TfTree()
        {
            clear();
        }

        //! Copy assigment operator.
        /*!
         * Creates a deep copy of the tree.
         * @param eq tree to be copied.
         * @return reference to *this.
         */
        TfTree &operator=(const TfTree &eq)
        {
            nodes = eq.nodes;
            root_node_key = eq.root_node_key;
            return *this;
        }

        //! Move assigment operator.
        /*!
         * Moves content of \p mv to a new tree. The root node is than copied back to keep \p mv valid.
         * @param mv tree to be moved from.
         * @return reference to *this.
         */
        TfTree &operator=(TfTree &&mv) noexcept
        {
            nodes = std::move(mv.nodes);
            root_node_key = std::move(mv.root_node_key);
            mv.insertRoot(root_node_key);
            return *this;
        }

        //! Checks for an empty tree.
        /*!
         * Since there always should be the root node, a valid tree should never be empty. Useful for checking if the tree became empty by exception.
         * @return False if there are no nodes in the tree, true otherwise.
         */
        [[nodiscard]] bool empty() const
        {
            return nodes.empty();
        }

        //! Returns number of the nodes in the tree.
        /*!
         *
         * @return number of the nodes in the tree.
         */
        [[nodiscard]] size_t size() const
        {
            return nodes.size();
        }

        //! Clears the tree leaving only the root unchanged.
        void clear()
        {
            std::vector<KeyType> child_keys;
            for (auto c : nodes.at(root_node_key).children())
                child_keys.push_back(c->key());
            for (auto k : child_keys)
                eraseSubtree(k);
        }

        //! Inserts a new node into the tree.
        /*!
         *
         * @tparam Tf type of the transformation.
         * @param key key of the new node.
         * @param tf transformation from the parent node to the new node.
         * @param parent key of the parent node.
         * @return true on success, false otherwise.
         */
        template<typename Tf>
        bool insert(const KeyType &key, Tf &&tf, const KeyType &parent)
        {
            auto it_parent = nodes.find(parent);
            if (it_parent == nodes.end())
                return false;
            return nodes.emplace(key, NodeType(key, tf, it_parent->second)).second;
        }

        //! Erases the node with given key and all its child-nodes recursively.
        /*!
         * The root node cannot be erased.
         * @param key key of the node to be erased.
         * @return true on success, false otherwise.
         */
        bool erase(const KeyType &key)
        {
            if (key == root_node_key)
                return false;
            auto n = nodes.find(key);
            if (n == nodes.end())
                return false;
            return eraseSubtree(key);
        }

        //! Checks whether a node with given key exists in the tree.
        /*!
         *
         * @param key key of the node to be searched for.
         * @return true if found, false otherwise.
         */
        bool contains(const KeyType &key) const
        {
            return nodes.find(key) != nodes.end();
        }

        //! Returns reference to the root node.
        /*!
         *
         * @return reference to the root node.
         */
        [[nodiscard]] const NodeType& root() const
        {
            return nodes.at(root_node_key);
        }

        //! Access a node with given \p key.
        /*!
         * If there is no node with the \p key in the tree, an exception is thrown.
         * @param key key of the accessed node.
         * @return reference to the accessed node.
         */
        NodeType& operator[](const KeyType& key)
        {
            if (!contains(key))
                throw std::out_of_range("The key does not exist in given TfTree.");
            return nodes[key];
        }

        //! Access a node with given \p key.
        /*!
         * If there is no node with the \p key in the tree, an exception is thrown.
         * @param key key of the accessed node.
         * @return reference to the accessed node.
         */
        NodeType& operator[](KeyType&& key)
        {
            if (!contains(key))
                throw std::out_of_range("The key does not exist in given TfTree.");
            return nodes[key];
        }

        //! Access a node with given \p key.
        /*!
         * If there is no node with the \p key in the tree, an exception is thrown.
         * @param key key of the accessed node.
         * @return reference to the accessed node.
         */
        NodeType& at(const KeyType& key)
        {
            return nodes.at(key);
        }

        //! Access a node with given \p key.
        /*!
         * If there is no node with the \p key in the tree, an exception is thrown.
         * @param key key of the accessed node.
         * @return reference to the accessed node.
         */
        const NodeType& at(const KeyType& key) const
        {
            return nodes.at(key);
        }

        //! Returns a chain of transformations between nodes.
        /*!
         *
         * @param from starting node.
         * @param to end node.
         * @return chain of transformations between \p from and \p to.
         */
        TfChain<TransformationType> tf(const KeyType& from, const KeyType& to) const
        {
            const NodeType *from_node = &nodes.at(from);
            const NodeType *to_node = &nodes.at(to);
            std::list<TransformationType> ret;
            size_t common_depth = std::min(from_node->depth(), to_node->depth());

            if (from_node->depth() > common_depth)
                for (;from_node->depth() != common_depth; from_node = from_node->parent())
                    ret.push_back(from_node->tf().inverted());
            auto insert_pos = ret.end();
            if (to_node->depth() > common_depth)
                for (; to_node->depth() != common_depth; to_node = to_node->parent())
                    insert_pos = ret.insert(insert_pos, to_node->tf());

            while (from_node->key() != to_node->key())
            {
                insert_pos = ret.insert(insert_pos, to_node->tf());
                ret.insert(insert_pos, from_node->tf().inverted());
                from_node = from_node->parent();
                to_node = to_node->parent();
            }

            return TfChain<TransformationType>(ret);
        }

    private:
        //! Recursively erases all children of given and and then the node itself.
        /*!
         *
         * @param key key of the node to be erased.
         * @return true on success, false otherwise.
         */
        bool eraseSubtree(const KeyType &key)
        {
            std::vector<KeyType> child_keys;
            for (auto c : nodes.at(key).children())
                child_keys.push_back(c->key());
            for (auto k : child_keys)
                eraseSubtree(k);

            return nodes.erase(key);
        }

        //! Creates a rood node with given key.
        /*!
         *
         * @param key key of the root node.
         */
        void insertRoot(const KeyType &key)
        {
            root_node_key = key;
            nodes.emplace(key, NodeType(key));
        }

        std::map<KeyType, NodeType> nodes;
        K root_node_key;
    };
}

#endif //ROBOTICTEMPLATELIBRARY_TFTREE_H
