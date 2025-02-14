#include "lidar_viewer/geometry/functions/Utilities.h"
#include "lidar_viewer/geometry/types/Box.h"
#include "lidar_viewer/geometry/types/Point.h"
#include "lidar_viewer/geometry/types/PointCloud.h"

#include <gtest/gtest.h>

namespace
{

// Utility function to compare two points (within a small tolerance)
bool pointsEqual(const lidar_viewer::geometry::types::Point3D<float>& p1,
                 const lidar_viewer::geometry::types::Point3D<float>& p2, float tolerance = 0.001f)
{
    return std::abs(p1[0] - p2[0]) < tolerance
    && std::abs(p1[1] - p2[1]) < tolerance
    && std::abs(p1[2] - p2[2]) < tolerance;
}

}

namespace lidar_viewer::tests::units
{

using lidar_viewer::geometry::types::Box;
using lidar_viewer::geometry::types::Point;
using lidar_viewer::geometry::types::Point3D;
using lidar_viewer::geometry::types::PointCloud3D;
using lidar_viewer::geometry::functions::valueToRGBByte;
using lidar_viewer::geometry::functions::mapValue;
using lidar_viewer::geometry::functions::midOf;
using lidar_viewer::geometry::functions::subdivisionOfBounbdingBox;
using lidar_viewer::geometry::functions::sphericalToEuclidean;
using lidar_viewer::geometry::functions::calculateBoundingBoxFromPointCloud;

TEST(ValueToRGBByteTest, BasicTest)
{
    unsigned int scalar = 100;
    float value = 1.5f;

    auto result = valueToRGBByte<uint8_t>(scalar, value);
    // 1.5 * 100 = 150, rounded to nearest byte value = 150
    ASSERT_EQ(result, 150);
}

TEST(ValueToRGBByteTest, DISABLED__LargeValueTest) // needs refactor TODO
{
    unsigned int scalar = 100;
    float value = 300.5f;

    auto result = valueToRGBByte<uint8_t>(scalar, value);
    // 300.5 * 100 = 30050, rounded to nearest byte value = 255 (since it's beyond byte max)
    ASSERT_EQ(result, 255);
}

TEST(ValueToRGBByteTest, ZeroValueTest)
{
    unsigned int scalar = 100;
    float value = 0.0f;

    auto result = valueToRGBByte<uint8_t>(scalar, value);
    // 0 * 100 = 0
    ASSERT_EQ(result, 0);
}

TEST(MapValueTest, BasicMappingTest)
{
    float aFirst = 0.0f;
    float bFirst = 0.0f;
    float upperNormScalar = 10.0f;
    float inVal = 5.0f;

    auto result = mapValue(aFirst, bFirst, upperNormScalar, inVal);
    // 0.0f + (5.0f - 0.0f) * 10.0f = 50.0f
    ASSERT_EQ(result, 50.0f);
}

TEST(MapValueTest, ReverseMappingTest)
{
    float aFirst = 0.0f;
    float bFirst = 100.0f;
    float upperNormScalar = 0.5f;
    float inVal = 10.0f;

    auto result = mapValue(aFirst, bFirst, upperNormScalar, inVal);
    // 100.0f + (10.0f - 0.0f) * 0.5f = 105.0f
    ASSERT_EQ(result, 105.0f);
}

TEST(SphericalToEuclideanTest, BasicTest)
{
    float zDepth = 10.0f;
    float rotationValueX = 0.5f;
    float rotationValueY = 0.5f;

    auto result = sphericalToEuclidean(zDepth, rotationValueX, rotationValueY);

    ASSERT_FLOAT_EQ(result[0], zDepth * std::cos(rotationValueX) * std::sin(rotationValueY));
    ASSERT_FLOAT_EQ(result[1], zDepth * std::sin(rotationValueX));
    ASSERT_FLOAT_EQ(result[2], zDepth * std::cos(rotationValueX) * std::cos(rotationValueY));
}

TEST(CalculateBoundingBoxFromPointCloudTest, BasicTest)
{
    PointCloud3D<float> pointCloud
    {
            Point3D<float>{{1.0f, 2.0f, 3.0f}},
            Point3D<float>{{4.0f, 5.0f, 6.0f}},
            Point3D<float>{{-1.0f, -2.0f, -3.0f}}
    };

    auto result = calculateBoundingBoxFromPointCloud(pointCloud);

    // The expected bounding box is from the min point (-1, -2, -3) to the max point (4, 5, 6)
    ASSERT_FLOAT_EQ(result.lo[0], -1.0f);
    ASSERT_FLOAT_EQ(result.lo[1], -2.0f);
    ASSERT_FLOAT_EQ(result.lo[2], -3.0f);
    ASSERT_FLOAT_EQ(result.hi[0], 4.0f);
    ASSERT_FLOAT_EQ(result.hi[1], 5.0f);
    ASSERT_FLOAT_EQ(result.hi[2], 6.0f);
}

TEST(SubdivisionOfBoundingBoxTest, AllSubdivisionsTest) {
    // Define a bounding box with known coordinates
    Box<Point3D<float>> box{Point3D<float>({4.0f, 4.0f, 4.0f}), Point3D<float>({0.0f, 0.0f, 0.0f})};

    // Iterate over all subdivision indices (0 to 7)
    for (size_t opIndex = 0; opIndex < 8; ++opIndex)
    {
        SCOPED_TRACE("Testing subdivision " + std::to_string(opIndex));

        // Apply the subdivision operation
        auto subdividedBox = subdivisionOfBounbdingBox(box, opIndex);

        // Verify the subdivision for each operation index:
        switch (opIndex)
        {
            case 0:
            {
                // Subdivision 0: divides the box in half along all axes
                ASSERT_TRUE(pointsEqual(subdividedBox.hi, Point3D<float>({4.0f, 4.0f, 4.0f})));
                ASSERT_TRUE(pointsEqual(subdividedBox.lo, Point3D<float>({2.0f, 2.0f, 2.0f})));
                break;
            }
            case 1:
            {
                // Subdivision 1: divides along x and y, but not z
                ASSERT_TRUE(pointsEqual(subdividedBox.hi, Point3D<float>({2.0f, 4.0f, 4.0f})));
                ASSERT_TRUE(pointsEqual(subdividedBox.lo, Point3D<float>({0.0f, 2.0f, 2.0f})));
                break;
            }
            case 2:
            {
                // Subdivision 2: divides along x and z, but not y
                ASSERT_TRUE(pointsEqual(subdividedBox.hi, Point3D<float>({4.0f, 2.0f, 4.0f})));
                ASSERT_TRUE(pointsEqual(subdividedBox.lo, Point3D<float>({2.0f, 0.0f, 2.0f})));
                break;
            }
            case 3:
            {
                // Subdivision 3: divides along x, y, and z
                ASSERT_TRUE(pointsEqual(subdividedBox.hi, Point3D<float>({2.0f, 2.0f, 4.0f})));
                ASSERT_TRUE(pointsEqual(subdividedBox.lo, Point3D<float>({0.0f, 0.0f, 2.0f})));
                break;
            }
            case 4:
            {
                // Subdivision 4: divides along x and y, but not z (upper half)
                ASSERT_TRUE(pointsEqual(subdividedBox.hi, Point3D<float>({4.0f, 4.0f, 2.0f})));
                ASSERT_TRUE(pointsEqual(subdividedBox.lo, Point3D<float>({2.0f, 2.0f, 0.0f})));
                break;
            }
            case 5:
            {
                // Subdivision 5: divides along y and z (lower half)
                ASSERT_TRUE(pointsEqual(subdividedBox.hi, Point3D<float>({2.0f, 4.0f, 2.0f})));
                ASSERT_TRUE(pointsEqual(subdividedBox.lo, Point3D<float>({0.0f, 2.0f, 0.0f})));
                break;
            }
            case 6:
            {
                // Subdivision 6: divides along x and z (lower half)
                ASSERT_TRUE(pointsEqual(subdividedBox.hi, Point3D<float>({4.0f, 2.0f, 2.0f})));
                ASSERT_TRUE(pointsEqual(subdividedBox.lo, Point3D<float>({2.0f, 0.0f, 0.0f})));
                break;
            }
            case 7:
            {
                // Subdivision 7: divides along y and z (upper half)
                ASSERT_TRUE(pointsEqual(subdividedBox.hi, Point3D<float>({2.0f, 2.0f, 2.0f})));
                ASSERT_TRUE(pointsEqual(subdividedBox.lo, Point3D<float>({0.0f, 0.0f, 0.0f})));
                break;
            }
        }
    }
}

TEST(MidOfTest, BasicTest) {
    Box<Point3D<float>> box{Point3D<float>{{10.0f, 10.0f, 10.0f}}, Point3D<float>{{0.0f, 0.0f, 0.0f}}};

    auto result = midOf(box, 0);
    ASSERT_FLOAT_EQ(result, 5.0f);

    result = midOf(box, 1);
    ASSERT_FLOAT_EQ(result, 5.0f);

    result = midOf(box, 2);
    ASSERT_FLOAT_EQ(result, 5.0f);
}

} // namespace lidar_viewer::tests::units
