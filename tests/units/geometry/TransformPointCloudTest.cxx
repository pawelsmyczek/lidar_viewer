#include "lidar_viewer/geometry/functions/TransformPointCloud.h"
#include "lidar_viewer/geometry/types/PointCloud.h"
#include "lidar_viewer/geometry/types/Transform3D.h"

#include <gtest/gtest.h>

#include <array>
#include <numbers>
#include <type_traits>

// API assumed by these tests:
//   template <typename CloudCoord, typename T>
//   PointCloud3D<CloudCoord> functions::transformPointCloud(const PointCloud3D<CloudCoord>& cloud,
//                                                           const Transform3D<T>& transform)
// returns a new cloud with `transform` applied to every point, in the same order. The math is done
// in the type of the transform, the result has the coordinate type of the cloud.

namespace lidar_viewer::tests::units
{

using lidar_viewer::geometry::functions::transformPointCloud;
using lidar_viewer::geometry::types::Point3D;
using lidar_viewer::geometry::types::PointCloud3D;
using lidar_viewer::geometry::types::Transform3D;

namespace
{

constexpr double Pi = std::numbers::pi;

template <typename T>
Point3D<T> point(T x, T y, T z)
{
    return Point3D<T>{std::array<T, 3>{x, y, z}};
}

template <typename P1, typename P2>
void expectPointNear(const P1& actual, const P2& expected, double tolerance = 1e-9)
{
    EXPECT_NEAR(actual[0], expected[0], tolerance) << "x";
    EXPECT_NEAR(actual[1], expected[1], tolerance) << "y";
    EXPECT_NEAR(actual[2], expected[2], tolerance) << "z";
}

} // namespace

TEST(TransformPointCloudTest, EmptyCloudGivesEmptyCloud)
{
    const PointCloud3D<double> cloud;

    const auto result = transformPointCloud(cloud, Transform3D<double>::fromTranslation(point(1.0, 2.0, 3.0)));

    EXPECT_TRUE(result.empty());
}

TEST(TransformPointCloudTest, IdentityKeepsAllPoints)
{
    const PointCloud3D<double> cloud{point(1.0, 2.0, 3.0), point(-4.0, 5.0, 0.5)};

    const auto result = transformPointCloud(cloud, Transform3D<double>::identity());

    ASSERT_EQ(result.size(), cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i)
    {
        expectPointNear(result[i], cloud[i]);
    }
}

TEST(TransformPointCloudTest, TranslationMovesEveryPoint)
{
    const PointCloud3D<double> cloud{point(0.0, 0.0, 0.0), point(1.0, 2.0, 3.0), point(-1.0, -1.0, -1.0)};

    const auto result = transformPointCloud(cloud, Transform3D<double>::fromTranslation(point(10.0, 20.0, 30.0)));

    ASSERT_EQ(result.size(), 3u);
    expectPointNear(result[0], point(10.0, 20.0, 30.0));
    expectPointNear(result[1], point(11.0, 22.0, 33.0));
    expectPointNear(result[2], point(9.0, 19.0, 29.0));
}

TEST(TransformPointCloudTest, RotationTurnsEveryPoint)
{
    const PointCloud3D<double> cloud{point(1.0, 0.0, 0.0), point(0.0, 1.0, 0.0), point(0.0, 0.0, 1.0)};

    const auto result = transformPointCloud(
        cloud, Transform3D<double>::fromAxisAngle(point(0.0, 0.0, 1.0), Pi / 2));

    ASSERT_EQ(result.size(), 3u);
    expectPointNear(result[0], point(0.0, 1.0, 0.0));
    expectPointNear(result[1], point(-1.0, 0.0, 0.0));
    expectPointNear(result[2], point(0.0, 0.0, 1.0));
}

TEST(TransformPointCloudTest, KeepsThePointOrder)
{
    PointCloud3D<double> cloud;
    for (int i = 0; i < 100; ++i)
    {
        cloud.push_back(point(static_cast<double>(i), 0.0, 0.0));
    }

    const auto result = transformPointCloud(cloud, Transform3D<double>::fromTranslation(point(0.5, 1.0, 2.0)));

    ASSERT_EQ(result.size(), cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i)
    {
        expectPointNear(result[i], point(static_cast<double>(i) + 0.5, 1.0, 2.0));
    }
}

TEST(TransformPointCloudTest, DoesNotModifyTheInputCloud)
{
    const PointCloud3D<double> cloud{point(1.0, 2.0, 3.0), point(4.0, 5.0, 6.0)};
    const auto copy = cloud;

    const auto result = transformPointCloud(cloud, Transform3D<double>::fromTranslation(point(1.0, 1.0, 1.0)));
    (void) result;

    ASSERT_EQ(cloud.size(), copy.size());
    for (size_t i = 0; i < cloud.size(); ++i)
    {
        expectPointNear(cloud[i], copy[i], 0.0);
    }
}

TEST(TransformPointCloudTest, TransformingByAComposedPoseEqualsTransformingStepByStep)
{
    const PointCloud3D<double> cloud{point(1.0, 2.0, 3.0), point(-2.0, 0.5, 4.0), point(0.0, 0.0, 0.0)};
    const auto a = Transform3D<double>::fromRollPitchYaw(0.3, -0.5, 1.1);
    const auto b = Transform3D<double>::fromTranslation(point(1.0, -2.0, 0.5));

    const auto composed = transformPointCloud(cloud, a * b);
    const auto stepwise = transformPointCloud(transformPointCloud(cloud, b), a);

    ASSERT_EQ(composed.size(), stepwise.size());
    for (size_t i = 0; i < composed.size(); ++i)
    {
        expectPointNear(composed[i], stepwise[i]);
    }
}

TEST(TransformPointCloudTest, InverseTransformRestoresTheCloud)
{
    const PointCloud3D<double> cloud{point(1.0, 2.0, 3.0), point(-2.0, 0.5, 4.0)};
    const auto pose = Transform3D<double>::fromAxisAngle(point(1.0, 2.0, 3.0), 0.7)
                      * Transform3D<double>::fromTranslation(point(1.5, -2.0, 0.25));

    const auto restored = transformPointCloud(transformPointCloud(cloud, pose), pose.inverse());

    ASSERT_EQ(restored.size(), cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i)
    {
        expectPointNear(restored[i], cloud[i]);
    }
}

TEST(TransformPointCloudTest, FloatCloudWithDoubleTransformKeepsTheCloudType)
{
    const PointCloud3D<float> cloud{point(1.0f, 0.0f, 0.0f), point(0.0f, 2.0f, 0.0f)};
    const auto pose = Transform3D<double>::fromAxisAngle(point(0.0, 0.0, 1.0), Pi / 2)
                      * Transform3D<double>::fromTranslation(point(1.0, 1.0, 1.0));

    const auto result = transformPointCloud(cloud, pose);

    static_assert(std::is_same_v<std::remove_const_t<decltype(result)>, PointCloud3D<float>>);
    ASSERT_EQ(result.size(), 2u);
    // translation first, then the turn around z: (x, y, z) -> (-y, x, z)
    expectPointNear(result[0], point(-1.0, 2.0, 1.0), 1e-5);
    expectPointNear(result[1], point(-3.0, 1.0, 1.0), 1e-5);
}

TEST(TransformPointCloudTest, FloatCloudWithFloatTransform)
{
    const PointCloud3D<float> cloud{point(1.0f, 2.0f, 3.0f)};

    const auto result = transformPointCloud(cloud, Transform3D<float>::fromTranslation(point(1.0f, 1.0f, 1.0f)));

    ASSERT_EQ(result.size(), 1u);
    expectPointNear(result[0], point(2.0f, 3.0f, 4.0f), 1e-6);
}

} // namespace lidar_viewer::tests::units
