#ifndef LIDAR_VIEWER_OCTREE_H
#define LIDAR_VIEWER_OCTREE_H

#include "OctreeIterator.h"

#include <array>
#include <algorithm>

namespace lidar_viewer::geometry::types
{

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

    OctreeNode* operator [] (size_t idx)
    {
        return children[idx];
    }

    OctreeNode*& at(size_t idx)
    {
        return children[idx];
    }

    bool isDivided()
    {
        return std::any_of(children.begin(), children.end(), [](auto & child) { return child != nullptr; } );
    }

    bool hasChild(const size_t id)
    {
        return children[id] != nullptr;
    }

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

template <typename ContainerType, typename KeyType>
struct Octree
{
    using NodeType = OctreeNode<ContainerType, KeyType>;
    friend struct OctreeDfsIterator<Octree>;

    using Iterator = OctreeDfsIterator<Octree>;


    Octree(KeyType initKey_, const size_t depth_)
            : root{new NodeType{initKey_}}
            , initKey{initKey_}
            , depth{depth_}
    {

    }

    explicit Octree(KeyType initKey_)
            : root{new NodeType{initKey_}}
            , initKey{initKey_}
            , depth{1u}
    {}

    virtual ~Octree() { deleteTree(); }

    NodeType* getRootNode() { return root; }
    size_t getDepth() { return depth; }

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

    void deleteNode(NodeType& node)
    {
        for (size_t i = 0u; i < 8; ++i)
        {
            deleteNodeChild(node, i);
        }
    }
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
