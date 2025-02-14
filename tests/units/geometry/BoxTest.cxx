#include "lidar_viewer/geometry/types/Box.h"
#include "lidar_viewer/geometry/types/Point.h"

#include <gtest/gtest.h>


namespace lidar_viewer::tests::units
{

using lidar_viewer::geometry::types::Point;
using lidar_viewer::geometry::types::Box;

// Test Box Construction
TEST(BoxTest, ConstructorInitialization)
{
    Point<int, 3> hi{{5, 5, 5}};
    Point<int, 3> lo{{1, 1, 1}};

    Box<Point<int, 3>> box{hi, lo};

    ASSERT_EQ(box.hi[0], 5);
    ASSERT_EQ(box.hi[1], 5);
    ASSERT_EQ(box.hi[2], 5);

    ASSERT_EQ(box.lo[0], 1);
    ASSERT_EQ(box.lo[1], 1);
    ASSERT_EQ(box.lo[2], 1);
}

TEST(BoxTest, DistanceInside)
{
    Box<Point<float, 3>> box{Point<float, 3>{{5.f, 5.f, 5.f}}, Point<float, 3>{{1.f, 1.f, 1.f}}};
    Point<float, 3> p{{3.f, 3.f, 3.f}};

    ASSERT_FLOAT_EQ(box.distance(p), 0.f);  // Inside the box, distance should be zero
}

TEST(BoxTest, DistanceOutside)
{
    Box<Point<double, 3>> box{Point<double, 3>{{5.0, 5.0, 5.0}}, Point<double, 3>{{ 1.0, 1.0, 1.0 }}};
    Point<double, 3> p{{7.0, 7.0, 7.0}};

    double expected_distance = std::sqrt(3.0 * std::pow(2.0, 2));  // sqrt(3 * 4) = sqrt(12)
    ASSERT_DOUBLE_EQ(box.distance(p), expected_distance);
}

TEST(BoxTest, DistanceOnEdge)
{
    Box<Point<float, 3>> box{Point<float, 3>{{5.f, 5.f, 5.f}}, Point<float, 3>{{1.f, 1.f, 1.f}}};
    Point<float, 3> p{{5.f, 3.f, 3.f}};

    ASSERT_FLOAT_EQ(box.distance(p), 0.f);  // On the edge, distance should be zero
}

TEST(BoxTest, ContainsInside)
{
    Box<Point<float, 3>> box{Point<float, 3>{{5.f, 5.f, 5.f}}, Point<float, 3>{{1.f, 1.f, 1.f}}};
    Point<float, 3> p{{3.f, 3.f, 3.f}};

    ASSERT_TRUE(box.contains(p));
}

TEST(BoxTest, ContainsOutside)
{
    Box<Point<float, 3>> box{Point<float, 3>{{5.f, 5.f, 5.f}}, Point<float, 3>{{1.f, 1.f, 1.f}}};
    Point<float, 3> p{{6.f, 6.f, 6.f}};

    ASSERT_FALSE(box.contains(p));
}

TEST(BoxTest, ContainsOnEdge)
{
    Box<Point<float, 3>> box{Point<float, 3>{{5.f, 5.f, 5.f}}, Point<float, 3>{{1.f, 1.f, 1.f}}};
    Point<float, 3> p({5.f, 3.f, 3.f});

    ASSERT_TRUE(box.contains(p));
}

} // namespace lidar_viewer::tests::units