#include "lidar_viewer/geometry/types/Point.h"
#include "lidar_viewer/geometry/types/Vector.h"

#include <gtest/gtest.h>

#include <type_traits>

namespace lidar_viewer::tests::units
{

using lidar_viewer::geometry::types::Point;
using lidar_viewer::geometry::types::Point3D;
using lidar_viewer::geometry::types::toPoint;
using lidar_viewer::geometry::types::toVector;
using lidar_viewer::geometry::types::Vector3D;
using lidar_viewer::geometry::types::VectorOf;

TEST(VectorTest, ToVectorKeepsTheCoordinates)
{
    const Point3D<double> p{{1.5, -2.0, 3.25}};

    const auto v = toVector<double>(p);

    EXPECT_DOUBLE_EQ(v[0], 1.5);
    EXPECT_DOUBLE_EQ(v[1], -2.0);
    EXPECT_DOUBLE_EQ(v[2], 3.25);
}

TEST(VectorTest, ToVectorConvertsTheCoordinateType)
{
    const Point3D<float> p{{1.5f, -2.0f, 3.25f}};

    const auto v = toVector<double>(p);

    static_assert(std::is_same_v<std::remove_const_t<decltype(v)>, Vector3D<double>>);
    EXPECT_DOUBLE_EQ(v[0], 1.5);
    EXPECT_DOUBLE_EQ(v[1], -2.0);
    EXPECT_DOUBLE_EQ(v[2], 3.25);
}

TEST(VectorTest, ToPointConvertsTheCoordinateType)
{
    const Vector3D<double> v{{1.5, -2.0, 3.25}};

    const auto p = toPoint<float>(v);

    static_assert(std::is_same_v<std::remove_const_t<decltype(p)>, Point3D<float>>);
    EXPECT_FLOAT_EQ(p[0], 1.5f);
    EXPECT_FLOAT_EQ(p[1], -2.0f);
    EXPECT_FLOAT_EQ(p[2], 3.25f);
}

TEST(VectorTest, ToPointTruncatesWhenConvertingToIntegers)
{
    const Vector3D<double> v{{1.9, -2.9, 3.5}};

    const auto p = toPoint<int>(v);

    EXPECT_EQ(p[0], 1);
    EXPECT_EQ(p[1], -2);
    EXPECT_EQ(p[2], 3);
}

TEST(VectorTest, ToVectorAndToPointRoundTrip)
{
    const Point3D<float> p{{1.5f, -2.0f, 3.25f}};

    const auto back = toPoint<float>(toVector<double>(p));

    EXPECT_FLOAT_EQ(back[0], p[0]);
    EXPECT_FLOAT_EQ(back[1], p[1]);
    EXPECT_FLOAT_EQ(back[2], p[2]);
}

TEST(VectorTest, ToVectorAndToPointWorkAtCompileTime)
{
    constexpr Point3D<float> p{{1.5f, 2.0f, 3.0f}};
    constexpr auto v = toVector<double>(p);
    static_assert(v[0] == 1.5 && v[1] == 2.0 && v[2] == 3.0);

    constexpr auto back = toPoint<float>(v);
    static_assert(back[0] == 1.5f && back[1] == 2.0f && back[2] == 3.0f);
    SUCCEED();
}

TEST(VectorTest, VectorOfAcceptsExactlyAVectorOfTheGivenTypeAndSize)
{
    static_assert(VectorOf<Vector3D<double>, double, 3>);
    static_assert(VectorOf<Point3D<double>, double, 3>);           // a Point is a Vector
    static_assert(VectorOf<Point<float, 5>, float, 5>);

    static_assert(!VectorOf<Vector3D<double>, double, 2>);         // wrong size
    static_assert(!VectorOf<Vector3D<float>, double, 3>);          // wrong coordinate type
    static_assert(!VectorOf<const Vector3D<double>, double, 3>);   // no qualifiers
    static_assert(!VectorOf<double, double, 3>);                   // not a vector at all
    SUCCEED();
}

} // namespace lidar_viewer::tests::units
