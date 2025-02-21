#include "lidar_viewer/geometry/functions/DownSample.h"

#include "lidar_viewer/geometry/types/PointCloud.h"
#include "lidar_viewer/geometry/types/Point.h"

#include <gtest/gtest.h>


namespace lidar_viewer::tests::units
{

using PointCloud3Dfloat = lidar_viewer::geometry::types::PointCloud3D<float>;
using Point3Dfloat = lidar_viewer::geometry::types::Point3D<float>;

using lidar_viewer::geometry::functions::downSample;

TEST(DownSampleTest, EmptyInput)
{
    PointCloud3Dfloat pointCloud;
    const auto result = downSample(pointCloud, 1.0f);
    EXPECT_TRUE(result.empty());
}

TEST(DownSampleTest, SmallInput)
{
    PointCloud3Dfloat pointCloud { Point3Dfloat{{0, 0, 0}},
                                     Point3Dfloat{{1, 1, 1}},
                                     Point3Dfloat{{2, 2, 2}},
                                     Point3Dfloat{{3, 3, 3}},
                                     Point3Dfloat{{4, 4, 4}},
                                     Point3Dfloat{{5, 5, 5}} };
    const auto result = downSample(pointCloud, 1.0f);
    EXPECT_TRUE(result.empty());
}

TEST(DownSampleTest, BasicDownsampling)
{
    PointCloud3Dfloat pointCloud
    {
            Point3Dfloat{{0, 0, 0}}, Point3Dfloat{{0.1, 0.1, 0.1}}, Point3Dfloat{{0.2, 0.2, 0.2}},
            Point3Dfloat{{1, 1, 1}}, Point3Dfloat{{1.1, 1.1, 1.1}}, Point3Dfloat{{2, 2, 2}}, Point3Dfloat{{3, 3, 3}}
    };
    float voxelSize = 1.0f;
    const auto result = downSample(pointCloud, voxelSize);
    EXPECT_EQ(result.size(), 3); // Expect 3 unique voxel centers
}

TEST(DownSampleTest, PrecisionTest)
{
    PointCloud3Dfloat pointCloud = {
            Point3Dfloat{{0.05, 0.05, 0.05}},
            Point3Dfloat{{0.1, 0.1, 0.1}},
            Point3Dfloat{{0.15, 0.15, 0.15}},
            Point3Dfloat{{1.05, 1.05, 1.05}},
            Point3Dfloat{{1.1, 1.1, 1.1}},
            Point3Dfloat{{1.15, 1.15, 1.15}},
            Point3Dfloat{{1.12, 1.12, 1.12}}
    };
    float voxelSize = 0.5f;
    const auto result = downSample(pointCloud, voxelSize);
    EXPECT_EQ(result.size(), 3); // Expect 3 unique voxel centers
}

} // namespace lidar_viewer::tests::units
