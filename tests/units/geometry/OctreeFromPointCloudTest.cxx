#include "lidar_viewer/geometry/types/OctreeFromPointCloud.h"
#include "lidar_viewer/geometry/types/PointCloud.h"
#include "lidar_viewer/linalg/Point.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

namespace lidar_viewer::tests::units
{
namespace
{

using Point3Df = linalg::Point3D<float>;

Point3Df point3(const float x, const float y, const float z)
{
    return Point3Df{std::array<float, 3>{x, y, z}};
}

/// distance between two points, computed in double
double distanceBetween(const Point3Df& a, const Point3Df& b)
{
    double sum = 0.0;
    for (size_t i = 0; i < 3; ++i)
    {
        const double d = static_cast<double>(a[i]) - static_cast<double>(b[i]);
        sum += d * d;
    }
    return std::sqrt(sum);
}

/// reference implementation: the distance to the closest point of the cloud, infinity for an empty cloud
double bruteForceNearestDistance(const geometry::types::PointCloud3D<float>& cloud, const Point3Df& query)
{
    double best = std::numeric_limits<double>::infinity();
    for (const auto& p : cloud)
    {
        best = std::min(best, distanceBetween(p, query));
    }
    return best;
}

geometry::types::PointCloud3D<float> randomCloud(const size_t size, const float extent, const unsigned seed)
{
    std::mt19937 generator{seed};
    std::uniform_real_distribution<float> coordinate{-extent, extent};
    geometry::types::PointCloud3D<float> cloud;
    for (size_t i = 0; i < size; ++i)
    {
        cloud.push_back(point3(coordinate(generator), coordinate(generator), coordinate(generator)));
    }
    return cloud;
}

} // namespace

using BaseOctree = geometry::types::OctreeFromPointCloud<linalg::Point3D<float>>;

class OctreeFromPointCloudTest : public ::testing::Test {
protected:
    OctreeFromPointCloudTest()
    : pointCloud {
        linalg::Point3D<float>{{0.1f, 0.2f, 0.3f}},
        linalg::Point3D<float>{{0.4f, 0.5f, 0.6f}},
        linalg::Point3D<float>{{0.7f, 0.8f, 0.9f}}
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

// ---------------------------------------------------------------- nearest neighbour search

TEST_F(OctreeFromPointCloudTest, GetPointCloudReturnsTheIndexedCloud)
{
    BaseOctree octree{pointCloud, 8u};

    EXPECT_EQ(&octree.getPointCloud(), &pointCloud);
}

TEST_F(OctreeFromPointCloudTest, NearestOfAnEmptyOctreeIsNothing)
{
    BaseOctree octree{pointCloud, 8u, false};

    EXPECT_FALSE(octree.nearest(point3(0.5f, 0.5f, 0.5f)).has_value());
}

TEST_F(OctreeFromPointCloudTest, NearestFindsTheClosestPoint)
{
    BaseOctree octree{pointCloud, 32u};

    const auto result = octree.nearest(point3(0.5f, 0.5f, 0.5f));

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->index, 1u);   // (0.4, 0.5, 0.6)
    EXPECT_NEAR(result->distance, std::sqrt(0.02f), 1e-6f);
}

TEST_F(OctreeFromPointCloudTest, NearestOfAPointFromTheCloudIsItself)
{
    BaseOctree octree{pointCloud, 32u};

    for (size_t i = 0; i < pointCloud.size(); ++i)
    {
        const auto result = octree.nearest(pointCloud[i]);

        ASSERT_TRUE(result.has_value());
        EXPECT_EQ(result->index, i);
        EXPECT_FLOAT_EQ(result->distance, 0.0f);
    }
}

TEST_F(OctreeFromPointCloudTest, NearestOfASinglePointCloud)
{
    const geometry::types::PointCloud3D<float> single{point3(1.0f, 2.0f, 3.0f)};
    BaseOctree octree{single, 8u};

    const auto result = octree.nearest(point3(1.0f, 2.0f, 5.0f));

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->index, 0u);
    EXPECT_FLOAT_EQ(result->distance, 2.0f);
}

TEST_F(OctreeFromPointCloudTest, QueryOutsideTheOctreeStillFindsTheNearestPoint)
{
    BaseOctree octree{pointCloud, 32u};

    const auto result = octree.nearest(point3(5.0f, 5.0f, 5.0f));

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->index, 2u);   // (0.7, 0.8, 0.9)
    EXPECT_NEAR(result->distance, distanceBetween(pointCloud[2], point3(5.0f, 5.0f, 5.0f)), 1e-5);
}

TEST_F(OctreeFromPointCloudTest, MaxDistanceLimitsTheSearchInclusively)
{
    const geometry::types::PointCloud3D<float> cloud{point3(0.0f, 0.0f, 0.0f), point3(1.0f, 0.0f, 0.0f)};
    BaseOctree octree{cloud, 8u};
    const auto query = point3(3.0f, 0.0f, 0.0f);   // the nearest point (1, 0, 0) is exactly 2 away

    EXPECT_FALSE(octree.nearest(query, 1.99f).has_value());
    const auto atTheLimit = octree.nearest(query, 2.0f);
    ASSERT_TRUE(atTheLimit.has_value());
    EXPECT_EQ(atTheLimit->index, 1u);
    EXPECT_FLOAT_EQ(atTheLimit->distance, 2.0f);
    EXPECT_TRUE(octree.nearest(query, 100.0f).has_value());
}

TEST_F(OctreeFromPointCloudTest, ZeroMaxDistanceFindsOnlyAnExactMatch)
{
    BaseOctree octree{pointCloud, 32u};

    EXPECT_FALSE(octree.nearest(point3(0.5f, 0.5f, 0.5f), 0.0f).has_value());
    const auto exact = octree.nearest(pointCloud[1], 0.0f);
    ASSERT_TRUE(exact.has_value());
    EXPECT_EQ(exact->index, 1u);
}

TEST_F(OctreeFromPointCloudTest, NegativeMaxDistanceFindsNothing)
{
    BaseOctree octree{pointCloud, 32u};

    EXPECT_FALSE(octree.nearest(pointCloud[0], -1.0f).has_value());
}

TEST_F(OctreeFromPointCloudTest, OnlyInsertedPointsAreSearched)
{
    BaseOctree octree{pointCloud, 32u, false};
    octree.insert(0);   // (0.1, 0.2, 0.3), although point 2 is closer to the query

    const auto result = octree.nearest(point3(0.7f, 0.8f, 0.9f));

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->index, 0u);
}

TEST_F(OctreeFromPointCloudTest, NearestWorksOnAConstOctree)
{
    const BaseOctree octree{pointCloud, 32u};

    const auto result = octree.nearest(point3(0.5f, 0.5f, 0.5f));

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->index, 1u);
}

TEST_F(OctreeFromPointCloudTest, NearestWorksWhenTheOctreeIsOnlyTheRoot)
{
    BaseOctree octree{pointCloud, 1u};

    const auto result = octree.nearest(point3(0.5f, 0.5f, 0.5f));

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->index, 1u);
}

TEST(OctreeNearestTest, DuplicatePointsGiveDistanceZero)
{
    const geometry::types::PointCloud3D<float> cloud{point3(1.0f, 1.0f, 1.0f), point3(1.0f, 1.0f, 1.0f),
                                                    point3(2.0f, 2.0f, 2.0f)};
    BaseOctree octree{cloud, 16u};

    const auto result = octree.nearest(point3(1.0f, 1.0f, 1.0f));

    ASSERT_TRUE(result.has_value());
    EXPECT_FLOAT_EQ(result->distance, 0.0f);
    EXPECT_LT(result->index, 2u);
}

TEST(OctreeNearestTest, NearestWorksWithDoubleCoordinates)
{
    using Point = linalg::Point3D<double>;
    const geometry::types::PointCloud3D<double> cloud{Point{{0.0, 0.0, 0.0}}, Point{{10.0, 0.0, 0.0}}};
    geometry::types::OctreeFromPointCloud<Point> octree{cloud, 8u};

    const auto result = octree.nearest(Point{{7.0, 0.0, 0.0}});

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->index, 1u);
    EXPECT_DOUBLE_EQ(result->distance, 3.0);
}

TEST(OctreeNearestTest, MatchesBruteForceForRandomCloudsAtEveryDepth)
{
    const auto cloud = randomCloud(2000, 10.0f, 12345u);
    const auto queries = randomCloud(500, 15.0f, 54321u);   // part of them lies outside of the cloud's box

    for (const size_t depth : {1u, 2u, 8u, 32u, 128u})
    {
        BaseOctree octree{cloud, depth};
        for (const auto& query : queries)
        {
            const auto result = octree.nearest(query);

            ASSERT_TRUE(result.has_value()) << "depth " << depth;
            EXPECT_NEAR(result->distance, bruteForceNearestDistance(cloud, query), 1e-4) << "depth " << depth;
            // the reported index must really be a point at the reported distance
            EXPECT_NEAR(distanceBetween(cloud[result->index], query), result->distance, 1e-4) << "depth " << depth;
        }
    }
}

TEST(OctreeNearestTest, MaxDistanceMatchesBruteForceForRandomClouds)
{
    const auto cloud = randomCloud(1000, 10.0f, 777u);
    const auto queries = randomCloud(300, 12.0f, 888u);
    const float maxDistance = 1.0f;
    BaseOctree octree{cloud, 32u};

    for (const auto& query : queries)
    {
        const auto expected = bruteForceNearestDistance(cloud, query);
        const auto result = octree.nearest(query, maxDistance);

        if (expected < maxDistance - 1e-4)
        {
            ASSERT_TRUE(result.has_value());
            EXPECT_NEAR(result->distance, expected, 1e-4);
        }
        else if (expected > maxDistance + 1e-4)
        {
            EXPECT_FALSE(result.has_value());
        }
    }
}

TEST(OctreeNearestTest, ClusteredCloudWithAFarOutlier)
{
    auto cloud = randomCloud(500, 0.001f, 4242u);   // a tight cluster around the origin
    cloud.push_back(point3(100.0f, 100.0f, 100.0f));
    BaseOctree octree{cloud, 32u};

    const auto nearOutlier = octree.nearest(point3(99.0f, 100.0f, 100.0f));
    ASSERT_TRUE(nearOutlier.has_value());
    EXPECT_EQ(nearOutlier->index, 500u);
    EXPECT_NEAR(nearOutlier->distance, 1.0f, 1e-4f);

    const auto nearCluster = octree.nearest(point3(0.5f, 0.0f, 0.0f));
    ASSERT_TRUE(nearCluster.has_value());
    EXPECT_NEAR(nearCluster->distance, bruteForceNearestDistance(cloud, point3(0.5f, 0.0f, 0.0f)), 1e-4);
    EXPECT_LT(nearCluster->index, 500u);
}
}