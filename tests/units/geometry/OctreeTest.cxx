#include "lidar_viewer/geometry/types/Octree.h"

#include <gtest/gtest.h>


namespace lidar_viewer::tests::units
{

using lidar_viewer::geometry::types::Octree;
using lidar_viewer::geometry::types::OctreeNode;
using TestKeyType = int; // Replace with appropriate key type
using TestContainerType = std::vector<int>; // Replace with appropriate container type
using TestOctreeNode = OctreeNode<TestContainerType, TestKeyType>;
using TestOctree = Octree<TestContainerType, TestKeyType>;

TEST(OctreeNodeTest, DefaultConstructor)
{
    TestOctreeNode node;
    EXPECT_FALSE(node.isDivided());
}

TEST(OctreeNodeTest, KeyAssignment)
{
    TestKeyType key = 42;
    TestOctreeNode node(key);
    EXPECT_EQ(node.getKey(), key);
}

TEST(OctreeNodeTest, ChildNodeCreation)
{
    TestOctreeNode node(1);
    node.at(0) = new TestOctreeNode(2);
    EXPECT_TRUE(node.hasChild(0));
    EXPECT_EQ(node[0]->getKey(), 2);
    delete node.at(0);
}

TEST(OctreeTest, Initialization)
{
    TestOctree octree(10, 3);
    EXPECT_EQ(octree.getRootNode()->getKey(), 10);
}

TEST(OctreeTest, NodeInsertion)
{
    TestOctree octree(0, 3);
    auto* root = octree.getRootNode();
    root->at(0) = new TestOctreeNode(5);
    EXPECT_TRUE(root->hasChild(0));
    EXPECT_EQ(root->at(0)->getKey(), 5);
    // nodes deleted at octree destruction
}

TEST(OctreeTest, DeleteNode)
{
    TestOctree octree(0, 3);
    auto* root = octree.getRootNode();
    root->at(1) = new TestOctreeNode(7);
    octree.deleteNodeChild(*root, 1);
    EXPECT_FALSE(root->hasChild(1));
}

TEST(OctreeTest, RecursiveInsertion)
{
    TestOctree octree(0, 3);
    auto keyComp = [](const TestKeyType&, size_t, bool) -> std::optional<TestKeyType>
    {
        return 5; // Simplified key comparator
    };

    auto* node = octree.createNodesRecursivelyAt(octree.getRootNode(), keyComp, 0, 3);
    EXPECT_NE(node, nullptr);
    EXPECT_EQ(node->getKey(), 5);
}
} // namespace lidar_viewer::tests::units