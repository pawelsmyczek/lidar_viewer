#include "lidar_viewer/ui/display/DisplayPointCloud.h"
#include "lidar_viewer/dev/CygLidarD1.h"

#include "lidar_viewer/geometry/types/Point.h"
#include "lidar_viewer/geometry/types/DepthFrameAttributes.h"
#include "lidar_viewer/geometry/types/ScreenRanges.h"
#include "lidar_viewer/geometry/types/PointCloud.h"
#include "lidar_viewer/geometry/functions/Utilities.h"
#include "lidar_viewer/geometry/functions/GetDepthImageToPointCloudProcessor.h"

using MapGlFloat3 = std::array<float, 3>;

namespace lidar_viewer::ui
{

bool displayPointCloud3D(const dev::CygLidarD1* lidar, lidar_viewer::ui::drawing::DrawPointColorFloatArr && drawPoint) noexcept
{
    using geometry::types::PointCloud3D;
    using geometry::functions::calculateBoundingBoxFromPointCloud;
    using geometry::functions::getDepthImageToPointCloudProcessor;
    using DepthImage3D = dev::CygLidarD1::PointCloud3D;
    if(!lidar)
    {
        return false;
    }
    if(lidar->failedToRead())
    {
        return false;
    }

    constexpr geometry::types::UintRange depthRange        = {51u, 3000u};

    constexpr geometry::types::DepthFrameAttributes depthFrameAttributes{dev::CygLidarD1::get3dFrameWindow(),
                                                                         depthRange, 60.f, 32.5f};

    geometry::types::ScreenRangeGl glScreenRange{};

    auto conversionFunction = getDepthImageToPointCloudProcessor<DepthImage3D>(depthFrameAttributes, glScreenRange);
    PointCloud3D<float> pointCloudV;
    lidar->use3dPointCloudWithArgs(conversionFunction, pointCloudV);

    for(auto point : pointCloudV)
    {
        MapGlFloat3 rgbValues{
                point[2] < .5f ? 2 * point[2] : 2 - 2 * point[2], // g
                point[2] < .5f ? 1 - 2 * point[2] : .0f, // r
                point[2] < .5f ? .0f : 2 * point[2] - 1 // b
        };
        drawPoint(point, rgbValues);
    }
    return true;
}

bool displayPointCloud2D(const dev::CygLidarD1* lidar, lidar_viewer::ui::drawing::DrawPointColorFloatArr && )
{

    if(!lidar)
    {
        return false;
    }
    lidar->use2dPointCloud([]([[maybe_unused]]const auto& pointCloud)
       {
           // TODO
       });
    return true;
}

}
