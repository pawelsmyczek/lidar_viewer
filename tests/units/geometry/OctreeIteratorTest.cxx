#include "lidar_viewer/geometry/types/OctreeIterator.h"
#include "lidar_viewer/geometry/types/Octree.h"
#include "lidar_viewer/geometry/types/PointCloud.h"
#include "lidar_viewer/geometry/types/OctreeFromPointCloud.h"
#include <gtest/gtest.h>

namespace lidar_viewer::tests::units
{

using KeyType = int;
using ContainterType = geometry::types::Indices ;

struct MockOctree
: public geometry::types::Octree<ContainterType, KeyType>
{
    ContainterType containter{};
    using Base = geometry::types::Octree<ContainterType, KeyType>;
    MockOctree(KeyType initKey_, const size_t depth)
    : Base(initKey_, depth)
    {}
    void insert(KeyType key)
    {
        auto comparisonFunction = [&key](const int& keyToCompare, size_t , bool ) -> geometry::types::ErrorOr<KeyType>
        {
            if(keyToCompare != key)
            {
                return key;
            }
            return std::nullopt;
        };
        createNodesRecursivelyAt<decltype(comparisonFunction)>(this->root, comparisonFunction, this->getKey(), getDepth());
    }

};

struct OctreeDfsIteratorTest
        : public ::testing::Test
{

    OctreeDfsIteratorTest()
            : octree{0, 8}
    {
        // Create a small octree structure
        octree.insert(1);
        octree.insert(2);
    }

protected:
    MockOctree octree;
};

TEST_F(OctreeDfsIteratorTest, IteratesOverNodesCorrectly)
{
    geometry::types::OctreeDfsIterator<MockOctree> it(&octree, octree.getDepth());
    geometry::types::OctreeDfsIterator<MockOctree> end(&octree, 0, nullptr);

    std::vector<MockOctree::NodeType*> visited;
    while (it != end)
    {
        visited.push_back(*it);
        ++it;
    }

    // Ensure all nodes were visited
    ASSERT_EQ(visited.size(), 3);
    EXPECT_EQ(visited[0], octree.getRootNode());
    EXPECT_EQ(visited[1], octree.getRootNode()->at(0));
    EXPECT_EQ(visited[2], octree.getRootNode()->at(0)->at(0));
}

TEST_F(OctreeDfsIteratorTest, HandlesEmptyOctree)
{
    MockOctree emptyOctree{0, 2};
    geometry::types::OctreeDfsIterator<MockOctree> it(&emptyOctree, 1);
    geometry::types::OctreeDfsIterator<MockOctree> end(&emptyOctree, 0, nullptr);

    ASSERT_NE(it, end);
    EXPECT_EQ(*it, emptyOctree.getRootNode());
    ++it;
    EXPECT_EQ(it, end);
}

TEST_F(OctreeDfsIteratorTest, EqualityOperatorWorks)
{
    geometry::types::OctreeDfsIterator<MockOctree> it1(&octree, 3);
    geometry::types::OctreeDfsIterator<MockOctree> it2(&octree, 3);
    geometry::types::OctreeDfsIterator<MockOctree> end(&octree, 0, nullptr);

    EXPECT_EQ(it1, it2);
    EXPECT_NE(it1, end);
}

}
