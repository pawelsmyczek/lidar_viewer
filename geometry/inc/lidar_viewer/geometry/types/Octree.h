#ifndef LIDAR_VIEWER_OCTREE_H
#define LIDAR_VIEWER_OCTREE_H

#include "OctreeIterator.h"

#include <array>
#include <algorithm>

namespace lidar_viewer::geometry::types
{

/// Node of an octree with up to eight children.
/// @tparam ContainerType payload stored in the node
/// @tparam KeyType key describing the space covered by the node, e.g. Box
template <typename ContainerType, typename KeyType>
struct OctreeNode
{
    OctreeNode() = default;
    explicit OctreeNode(const KeyType& key_ )
    : key{key_}
    {}
    OctreeNode(const OctreeNode& source)
    {
        for (unsigned int i = 0u; i < 8; ++i)
        {
            if(source.children[i] != nullptr)
            {
                children[i] = source.children[i]->clone();
            }
        }
    }

    OctreeNode& operator = (const OctreeNode& source)
    {
        for (unsigned int i = 0u; i < 8; ++i)
        {
            if(source.children[i] != nullptr)
            {
                children[i] = source.children[i]->clone();
            }
        }
        return *this;
    }

    /// child number `idx` (0-7) or nullptr when it does not exist
    OctreeNode* operator [] (size_t idx)
    {
        return children[idx];
    }

    /// reference to the child pointer number `idx` (0-7), allows attaching a child
    OctreeNode*& at(size_t idx)
    {
        return children[idx];
    }

    /// true if the node has at least one child
    bool isDivided()
    {
        return std::any_of(children.begin(), children.end(), [](auto & child) { return child != nullptr; } );
    }

    /// true if child number `id` (0-7) exists
    bool hasChild(const size_t id)
    {
        return children[id] != nullptr;
    }

    /// payload of the node
    ContainerType& getContainer()
    {
        return container;
    }

    const ContainerType& getContainer() const
    {
        return container;
    }

    ContainerType* getContainerPtr()
    {
        return container;
    }

    const ContainerType* getContainerPtr() const
    {
        return container;
    }

    /// key of the node
    KeyType& getKey()
    {
        return key;
    }

    const KeyType& getKey() const
    {
        return key;
    }

    KeyType* getKeyPtr()
    {
        return &key;
    }

    const KeyType* getKeyPtr() const
    {
        return &key;
    }

private:

    OctreeNode* clone ()
    {
        return new OctreeNode<ContainerType, KeyType>(*this);
    }

    KeyType key;
    ContainerType container{};
    std::array<OctreeNode*, 8> children{};
};

/// Octree owning a hierarchy of OctreeNode. Each node covers a region described by its key
/// and may be divided into eight children.
/// @tparam ContainerType payload stored in each node
/// @tparam KeyType key describing the space covered by a node, e.g. Box
template <typename ContainerType, typename KeyType>
struct Octree
{
    using NodeType = OctreeNode<ContainerType, KeyType>;
    friend struct OctreeDfsIterator<Octree>;

    using Iterator = OctreeDfsIterator<Octree>;


    /// @param initKey_ key of the root node
    /// @param depth_ maximal depth, halved on every level, so nodes are created
    ///               log2(depth_) levels below the root (e.g. 32 gives 5 levels)
    Octree(KeyType initKey_, const size_t depth_)
            : root{new NodeType{initKey_}}
            , initKey{initKey_}
            , depth{depth_}
    {

    }

    /// creates an octree with the default depth of 1 (root only)
    explicit Octree(KeyType initKey_)
            : root{new NodeType{initKey_}}
            , initKey{initKey_}
            , depth{1u}
    {}

    virtual ~Octree() { deleteTree(); }

    /// root node, owned by the octree
    NodeType* getRootNode() { return root; }
    size_t getDepth() { return depth; }

    /// depth first iteration over the nodes, starting at the root
    Iterator begin()
    {
        return Iterator{this, depth};
    }

    const Iterator begin() const
    {
        return Iterator{this, depth};
    }

    Iterator end()
    {
        return Iterator{this, 0, nullptr};
    }
    const Iterator end() const
    {
        return Iterator{this, 0, nullptr};
    }
    /// Walks down the tree, creating missing nodes, to the node responsible for a key.
    /// @param node node to start from
    /// @param keyComp callable `(const KeyType& key, size_t childIndex, bool divide)` returning an optional key
    ///        of the child `childIndex` (subdividing `key` when `divide` is true), or nothing if the
    ///        searched element does not belong to that child
    /// @param key key of `node`
    /// @param depth_ remaining depth, halved on every level; recursion stops when it is <= 1
    /// @return the node reached, nullptr if no child matches
    template<typename KeyComparatorF>
    NodeType* createNodesRecursivelyAt(NodeType* node,
                                  KeyComparatorF keyComp,
                                  const KeyType& key,
                                  size_t depth_)
    {
        auto retNode = node;
        if(depth_ <= 1)
        {
            return retNode;
        }
        for(unsigned int i = 0; i < 8; ++i)
        {
            if(node->at(i) != nullptr)
            {
                if(!keyComp(node->at(i)->getKey(), i, false))
                {
                    continue;
                }
            }
            else
            {
                auto subkey = keyComp(key, i, true);
                if(!subkey)
                {
                    continue;
                }
                node->at(i) = new NodeType{subkey.value()};
            }
            return createNodesRecursivelyAt(node->at(i), keyComp, node->at(i)->getKey(), depth_ >> 1);
        }
        return nullptr;
    }

    /// deletes the child number `id` of `node` together with its subtree
    void deleteNodeChild(NodeType& node, const size_t id)
    {
        if(node.hasChild(id))
        {
            auto child = node.at(id);
            deleteNode(*child);
            delete child;
            node.at(id) = nullptr;
        }
    }

    /// deletes all children of `node`, the node itself is kept
    void deleteNode(NodeType& node)
    {
        for (size_t i = 0u; i < 8; ++i)
        {
            deleteNodeChild(node, i);
        }
    }
    /// deletes all nodes except the root
    void deleteTree()
    {
        if(root)
        {
            deleteNode(*root);
        }
    }
protected:
    KeyType getKey() {return initKey; }

    NodeType* root;
    KeyType initKey;
    size_t depth{};
};

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_OCTREE_H
