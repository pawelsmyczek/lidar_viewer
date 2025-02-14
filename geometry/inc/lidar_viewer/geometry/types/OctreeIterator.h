#ifndef LIDAR_VIEWER_OCTREEITERATOR_H
#define LIDAR_VIEWER_OCTREEITERATOR_H

#include <stack>

namespace lidar_viewer::geometry::types
{

template <typename Octree>
struct OctreeDfsIterator
{
    using Node = Octree::NodeType;
    struct State
    {
        Node* node;
        size_t depth;
    };
    OctreeDfsIterator(Octree* octree_, size_t depth_, State* state_)
            : octree{octree_}
            , stack{}
            , state{state_}
            , depth{depth_}
    { }

    OctreeDfsIterator(Octree* octree_, size_t depth_)
    : octree{octree_}
    , stack{}
    , depth{depth_}
    {

        // pushing root node to stack
        State stack_entry;
        stack_entry.node = octree->getRootNode();
        stack_entry.depth = depth;

        stack.push(stack_entry);

        state = &stack.top();
    }

    bool operator==(const OctreeDfsIterator& other) const
    {
        if (this == &other) // same object
            return true;
        if (octree != other.octree) // refer to different octrees
            return false;
        if (!state && !other.state) // both are end iterators
            return true;
        if (depth == other.depth && depth &&
            other.depth)
            return true;
        return false;
    }

    bool operator!=(const OctreeDfsIterator& other) const
    {
        return !operator==(other);
    }

    Node* operator*() const
    {
        // return designated object
        return (octree && state) ? (state->node) : nullptr;
    }

    OctreeDfsIterator& operator++()
    {
        impl();
        return *this;
    }

    OctreeDfsIterator operator++(int)
    {
        auto tmp = *this;
        ++(*this);
        return tmp;
    }

private:
    void impl()
    {
        if (stack.empty())
        {
            return;
        }
        // get stack element
        auto stack_entry = stack.top();
        stack.pop();

        stack_entry.depth >>= 1;

        if (depth >= stack_entry.depth)
        {
            // current node
            auto node = stack_entry.node;

            // add all children to stack
            for (size_t child_idx = 0u; child_idx < 8; ++child_idx)
            {
                // if child exist
                if (node->hasChild(child_idx))
                {
                    // add child to stack
                    stack_entry.node = node->at(child_idx);
                    stack.push(stack_entry);

                }
            }
        }
        state = !stack.empty() ? &stack.top() : nullptr;
    }
    Octree* octree;
    std::stack<State> stack;
    State* state;
    size_t depth;
};

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_OCTREEITERATOR_H
