#include "lidar_viewer/geometry/types/Point.h"

#include <gtest/gtest.h>

namespace lidar_viewer::tests::units
{

using lidar_viewer::geometry::types::Point;

TEST(PointTest, ConstructorInitialization)
{
    std::array<int, 3> values {1, 2, 3};
    Point<int, 3> p(values);

    ASSERT_EQ(p[0], 1);
    ASSERT_EQ(p[1], 2);
    ASSERT_EQ(p[2], 3);
}

TEST(PointTest, BracketOperator)
{
    Point<float, 2> p({4.5f, 5.5f});

    ASSERT_FLOAT_EQ(p[0], 4.5);
    ASSERT_FLOAT_EQ(p[1], 5.5);
}

TEST(PointTest, AtMethod)
{
    Point<float, 2> p({7.1f, 8.2f});

    ASSERT_FLOAT_EQ(p.at(0), 7.1);
    ASSERT_FLOAT_EQ(p.at(1), 8.2);
}

TEST(PointTest, AdditionOperator)
{
    Point<int, 3> p1({1, 2, 3});
    Point<int, 3> p2({4, 5, 6});
    Point<int, 3> result = p1 + p2;

    ASSERT_EQ(result[0], 5);
    ASSERT_EQ(result[1], 7);
    ASSERT_EQ(result[2], 9);
}

TEST(PointTest, SubtractionOperator)
{
    Point<int, 3> p1({5, 7, 9});
    Point<int, 3> p2({1, 2, 3});
    Point<int, 3> result = p1 - p2;

    ASSERT_EQ(result[0], 4);
    ASSERT_EQ(result[1], 5);
    ASSERT_EQ(result[2], 6);
}

TEST(PointTest, AdditionAssignmentOperator)
{
    Point<int, 2> p1({1, 2});
    Point<int, 2> p2({3, 4});
    p1 += p2;

    ASSERT_EQ(p1[0], 4);
    ASSERT_EQ(p1[1], 6);
}

TEST(PointTest, DivisionOperator)
{
    Point<int, 2> p({10, 20});
    p = p / 2;

    ASSERT_EQ(p[0], 5);
    ASSERT_EQ(p[1], 10);
}

TEST(PointTest, DataMethod) {
    Point<int, 3> p({1, 2, 3});
    const int* data_ptr = p.data();

    ASSERT_EQ(data_ptr[0], 1);
    ASSERT_EQ(data_ptr[1], 2);
    ASSERT_EQ(data_ptr[2], 3);
}

TEST(PointTest, DefaultConstructor)
{
    Point<int, 2> p;
    // No direct check, but ensures default construction doesn't fail
}

// Test copy constructor
TEST(PointTest, CopyConstructor) {
    Point<int, 2> p1({1, 2});
    Point<int, 2> p2(p1);

    ASSERT_EQ(p2[0], 1);
    ASSERT_EQ(p2[1], 2);
}

// Test move constructor
TEST(PointTest, MoveConstructor) {
    Point<int, 2> p1({1, 2});
    Point<int, 2> p2(std::move(p1));

    ASSERT_EQ(p2[0], 1);
    ASSERT_EQ(p2[1], 2);
}

// Test copy assignment
TEST(PointTest, CopyAssignment) {
    Point<int, 2> p1({3, 4});
    Point<int, 2> p2;
    p2 = p1;

    ASSERT_EQ(p2[0], 3);
    ASSERT_EQ(p2[1], 4);
}

// Test move assignment
TEST(PointTest, MoveAssignment) {
    Point<int, 2> p1({3, 4});
    Point<int, 2> p2;
    p2 = std::move(p1);

    ASSERT_EQ(p2[0], 3);
    ASSERT_EQ(p2[1], 4);
}

} // namespace lidar_viewer::tests::units