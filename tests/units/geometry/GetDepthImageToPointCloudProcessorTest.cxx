#include "lidar_viewer/geometry/functions/GetDepthImageToPointCloudProcessor.h"
#include "lidar_viewer/geometry/types/Box.h"
#include "lidar_viewer/geometry/types/Point.h"
#include "lidar_viewer/geometry/types/PointCloud.h"

#include <gtest/gtest.h>


namespace lidar_viewer::tests::units
{

using namespace geometry::types;
using namespace geometry::functions;

struct ScreenRangeDummy
    : public ScreenRanges
{
    [[nodiscard]] FloatRange fullRangeX() const override
    {
        return glFullScreenRangeX;
    }

    [[nodiscard]] FloatRange fullRangeY() const override
    {
        return glFullScreenRangeY;
    }

    [[nodiscard]] FloatRange fullRangeZ() const override
    {
        return glFullScreenRangeZ;
    }

private:
    static constexpr FloatRange glFullScreenRangeX {0.0f, 1.0f};
    static constexpr FloatRange glFullScreenRangeY {0.0f, 1.0f};
    static constexpr FloatRange glFullScreenRangeZ {0.0f, 1.0f};
};

TEST(GetDepthImageToPointCloudProcessorTest, ValidConversionTest)
{
    DepthFrameAttributes depthAttributes{{5, 5},  // 5x5 frame
                                            {0.0f, 25.0f},
                                            45.0f,  // Some random rotation value
                                            45.0f};  // Some random rotation value

    ScreenRangeDummy screenRange;

    std::vector<float> frame3d {
            1.0f, 2.0f, 3.0f, 4.0f, 5.0f,
            6.0f, 7.0f, 8.0f, 9.0f, 10.0f,
            11.0f, 12.0f, 13.0f, 14.0f, 15.0f,
            16.0f, 17.0f, 18.0f, 19.0f, 20.0f,
            21.0f, 22.0f, 23.0f, 24.0f, 25.0f
    };

    // Get the processor function
    auto processor = getDepthImageToPointCloudProcessor<std::vector<float>>(depthAttributes, screenRange);

    PointCloud3D<float> pointCloud;
    processor(frame3d, pointCloud);

    // Check if the point cloud is populated correctly (expecting 10 points for a 5x5 grid)
    EXPECT_EQ(pointCloud.size(), 25);
}

TEST(GetDepthImageToPointCloudProcessorTest, DepthOutOfRangeTest)
{
    DepthFrameAttributes depthAttributes{{5, 5},  // 5x5 frame
                                                          {0.0f, 10.0f},
                                                          45.0f,  // Some random rotation value
                                                          45.0f};  // Some random rotation value

    ScreenRangeDummy screenRange;

    std::vector<float> frame3d {
            1.0f, 2.0f, 11.0f, 4.0f, 5.0f,  // 11.0f is out of range
            6.0f, 7.0f, 8.0f, 9.0f, 10.0f,  // These values are in range
            11.0f, 12.0f, 13.0f, 14.0f, 15.0f, // 11.0f is out of range
            6.0f, 7.0f, 8.0f, 9.0f, 10.0f, // These values are in range
            1.0f, 2.0f, 3.0f, 4.0f, 5.0f  // These values are in range
    };

    // Get the processor function
    auto processor = getDepthImageToPointCloudProcessor<std::vector<float>>(depthAttributes, screenRange);

    PointCloud3D<float> pointCloud;
    processor(frame3d, pointCloud);

    // Check that the point cloud size is 19 instead of 25, as 6 points are out of range
    EXPECT_EQ(pointCloud.size(), 19);
}

TEST(GetDepthImageToPointCloudProcessorTest, EmptyFrameTest)
{
    DepthFrameAttributes depthAttributes{{5, 5},  // 5x5 frame
                                         {0.0f, 10.0f},
                                         45.0f,  // Some random rotation value
                                         45.0f};  // Some random rotation value

    ScreenRangeDummy screenRange;

    std::vector<float> frame3d = {};

    auto processor = getDepthImageToPointCloudProcessor<std::vector<float>>(depthAttributes, screenRange);

    PointCloud3D<float> pointCloud;
    processor(frame3d, pointCloud);

    EXPECT_EQ(pointCloud.size(), 0);
}

TEST(GetDepthImageToPointCloudProcessorTest, LargeDepthFrameTest)
{
    DepthFrameAttributes depthAttributes{{1000, 1000},  // 1000x1000 frame
                                         {0.0f, 10.0f},
                                         45.0f,  // Some random rotation value
                                         45.0f};  // Some random rotation value

    ScreenRangeDummy screenRange;

    // Generate a large depth frame with some random values
    std::vector<float> frame3d(1000000, 5.0f);  // 1000x1000 frame with all depth values set to 5.0f

    // Get the processor function
    auto processor = getDepthImageToPointCloudProcessor<std::vector<float>>(depthAttributes, screenRange);

    PointCloud3D<float> pointCloud;
    processor(frame3d, pointCloud);

    // Verify that the point cloud has the expected number of points (1000x1000 = 1,000,000 points)
    EXPECT_EQ(pointCloud.size(), 1000000);
}

} // namespace lidar_viewer::tests::units