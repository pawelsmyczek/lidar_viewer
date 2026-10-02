#include "lidar_viewer/geometry/types/Point.h"

#include <gtest/gtest.h>

#include <type_traits>

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
    Point<int, 2>{};
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

TEST(PointTest, DefaultConstructedPointIsZero)
{
    const Point<double, 3> p3;
    const Point<int, 2> p2;
    const Point<float, 5> p5;

    for (size_t i = 0; i < 3; ++i) { EXPECT_EQ(p3[i], 0.0); }
    for (size_t i = 0; i < 2; ++i) { EXPECT_EQ(p2[i], 0); }
    for (size_t i = 0; i < 5; ++i) { EXPECT_EQ(p5[i], 0.0f); }
}

TEST(PointTest, ConstructionAndAccessWorkAtCompileTime)
{
    constexpr Point<int, 3> p{{1, 2, 3}};
    static_assert(p[0] == 1 && p[1] == 2 && p[2] == 3);
    static_assert(p.at(1) == 2);
    static_assert(*p.data() == 1);

    constexpr Point<int, 3> zero;
    static_assert(zero[0] == 0 && zero[1] == 0 && zero[2] == 0);
    SUCCEED();
}

TEST(PointTest, ConvertingConstructorCastsEveryCoordinate)
{
    const Point<float, 3> p{{1.5f, -2.0f, 3.25f}};

    const Point<double, 3> converted{p};

    EXPECT_DOUBLE_EQ(converted[0], 1.5);
    EXPECT_DOUBLE_EQ(converted[1], -2.0);
    EXPECT_DOUBLE_EQ(converted[2], 3.25);
}

TEST(PointTest, ConvertingConstructorTruncatesToIntegers)
{
    const Point<double, 2> p{{1.9, -2.9}};

    const Point<int, 2> converted{p};

    EXPECT_EQ(converted[0], 1);
    EXPECT_EQ(converted[1], -2);
}

TEST(PointTest, ConvertingConstructorDoesNotModifyTheSource)
{
    const Point<float, 3> p{{1.5f, -2.0f, 3.25f}};

    const Point<double, 3> converted{p};
    (void) converted;

    EXPECT_FLOAT_EQ(p[0], 1.5f);
    EXPECT_FLOAT_EQ(p[1], -2.0f);
    EXPECT_FLOAT_EQ(p[2], 3.25f);
}

TEST(PointTest, ConvertingConstructorWorksAtCompileTime)
{
    constexpr Point<float, 3> p{{1.5f, 2.0f, 3.0f}};
    constexpr Point<double, 3> converted{p};
    static_assert(converted[0] == 1.5 && converted[1] == 2.0 && converted[2] == 3.0);
    SUCCEED();
}

TEST(PointTest, ConvertingConstructorIsExplicitAndKeepsTheDimension)
{
    static_assert(std::is_constructible_v<Point<double, 3>, const Point<float, 3>&>);
    static_assert(!std::is_convertible_v<Point<float, 3>, Point<double, 3>>);          // explicit
    static_assert(!std::is_constructible_v<Point<float, 2>, const Point<float, 3>&>);  // other dimension
    SUCCEED();
}

TEST(PointTest, SameTypeCopyStillCopies)
{
    const Point<float, 3> p{{1.5f, -2.0f, 3.25f}};

    const Point<float, 3> copy{p};

    EXPECT_FLOAT_EQ(copy[0], 1.5f);
    EXPECT_FLOAT_EQ(copy[1], -2.0f);
    EXPECT_FLOAT_EQ(copy[2], 3.25f);
}

TEST(PointTest, AdditionAndSubtractionWorkOnConstPoints)
{
    const Point<int, 3> a({1, 2, 3});
    const Point<int, 3> b({10, 20, 30});

    const auto sum = a + b;
    const auto difference = b - a;

    EXPECT_EQ(sum[0], 11);
    EXPECT_EQ(sum[1], 22);
    EXPECT_EQ(sum[2], 33);
    EXPECT_EQ(difference[0], 9);
    EXPECT_EQ(difference[1], 18);
    EXPECT_EQ(difference[2], 27);
}

TEST(PointTest, AdditionAndSubtractionDoNotModifyTheOperands)
{
    const Point<int, 2> a({1, 2});
    const Point<int, 2> b({3, 4});

    const auto sum = a + b;
    const auto difference = a - b;
    (void) sum;
    (void) difference;

    EXPECT_EQ(a[0], 1);
    EXPECT_EQ(a[1], 2);
    EXPECT_EQ(b[0], 3);
    EXPECT_EQ(b[1], 4);
}

TEST(PointTest, SubtractAssignSubtractsInPlace)
{
    Point<int, 3> a({10, 20, 30});
    const Point<int, 3> b({1, 2, 3});

    auto& result = (a -= b);

    EXPECT_EQ(&result, &a);
    EXPECT_EQ(a[0], 9);
    EXPECT_EQ(a[1], 18);
    EXPECT_EQ(a[2], 27);
}

TEST(PointTest, ArithmeticWorksAtCompileTime)
{
    constexpr Point<int, 3> a{{1, 2, 3}};
    constexpr Point<int, 3> b{{10, 20, 30}};
    constexpr auto sum = a + b;
    constexpr auto difference = b - a;
    static_assert(sum[0] == 11 && sum[1] == 22 && sum[2] == 33);
    static_assert(difference[0] == 9 && difference[1] == 18 && difference[2] == 27);

    constexpr auto accumulated = [a]
    {
        Point<int, 3> p{{1, 1, 1}};
        p += a;
        p -= Point<int, 3>{{0, 1, 2}};
        return p;
    }();
    static_assert(accumulated[0] == 2 && accumulated[1] == 2 && accumulated[2] == 2);
    SUCCEED();
}

TEST(PointTest, ArithmeticWorksForOtherDimensions)
{
    const Point<double, 1> one1({2.0});
    const Point<double, 1> one2({3.0});
    EXPECT_DOUBLE_EQ((one1 + one2)[0], 5.0);

    const Point<int, 5> five1({1, 2, 3, 4, 5});
    const Point<int, 5> five2({5, 4, 3, 2, 1});
    const auto sum = five1 + five2;
    for (size_t i = 0; i < 5; ++i)
    {
        EXPECT_EQ(sum[i], 6) << "coordinate " << i;
    }
}

TEST(PointTest, ConvertingConstructorWorksForOtherDimensions)
{
    const Point<float, 1> one({1.5f});
    const Point<double, 1> convertedOne{one};
    EXPECT_DOUBLE_EQ(convertedOne[0], 1.5);

    const Point<double, 5> five({1.5, 2.5, 3.5, 4.5, 5.5});
    const Point<float, 5> convertedFive{five};
    for (size_t i = 0; i < 5; ++i)
    {
        EXPECT_FLOAT_EQ(convertedFive[i], static_cast<float>(five[i])) << "coordinate " << i;
    }
}

TEST(PointTest, DivisionStillDividesEveryCoordinateInPlace)
{
    Point<double, 3> p({2.0, 4.0, 6.0});

    auto& result = (p / 2);

    EXPECT_EQ(&result, &p);
    EXPECT_DOUBLE_EQ(p[0], 1.0);
    EXPECT_DOUBLE_EQ(p[1], 2.0);
    EXPECT_DOUBLE_EQ(p[2], 3.0);
}

} // namespace lidar_viewer::tests::units