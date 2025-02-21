#include "lidar_viewer/geometry/types/OctreeFromPointCloud.h"
#include "lidar_viewer/geometry/types/PointCloud.h"
#include "lidar_viewer/geometry/types/Point.h"

#include <gtest/gtest.h>

namespace lidar_viewer::tests::units
{
using BaseOctree = geometry::types::OctreeFromPointCloud<geometry::types::Point3D<float>>;

class OctreeFromPointCloudTest : public ::testing::Test {
protected:
    OctreeFromPointCloudTest()
    : pointCloud {
        geometry::types::Point3D<float>{{0.1f, 0.2f, 0.3f}},
        geometry::types::Point3D<float>{{0.4f, 0.5f, 0.6f}},
        geometry::types::Point3D<float>{{0.7f, 0.8f, 0.9f}}
    }
    {
    }

    geometry::types::PointCloud3D<float> pointCloud;
};

TEST_F(OctreeFromPointCloudTest, ConstructFromPointCloud)
{
    BaseOctree octree(pointCloud);
    EXPECT_FALSE(octree.getRootNode() == nullptr) << "Octree root should not be null";
}

TEST_F(OctreeFromPointCloudTest, ConstructWithDepth)
{
    size_t depth = 3;
    BaseOctree octree{pointCloud, depth};
    EXPECT_FALSE(octree.getRootNode() == nullptr);
    EXPECT_EQ(octree.getDepth(), depth);
}

TEST_F(OctreeFromPointCloudTest, InsertPoint)
{
    BaseOctree octree{pointCloud, 1u, false};
    auto node = octree.insert(0);
    EXPECT_NE(node, nullptr) << "Node should be created for inserted point";
EXPECT_FALSE(node->getContainer().empty()) << "Node should contain the inserted point index";
}

TEST_F(OctreeFromPointCloudTest, FillWithPointCloud)
{
    BaseOctree octree{pointCloud, 1u, false};
    octree.fillWithPointCloud();
    for (size_t i = 0; i < pointCloud.size(); ++i)
    {
        auto node = octree.insert(i);
        EXPECT_NE(node, nullptr) << "Each point should be inserted into the octree";
    }
}
}