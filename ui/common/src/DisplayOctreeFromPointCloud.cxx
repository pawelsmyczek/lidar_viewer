#include "lidar_viewer/ui/display/DisplayOctreeFromPointCloud.h"
#include "lidar_viewer/dev/CygLidarD1.h"

#include "lidar_viewer/geometry/types/Point.h"
#include "lidar_viewer/geometry/types/Box.h"
#include "lidar_viewer/geometry/types/PointCloud.h"
#include "lidar_viewer/geometry/types/OctreeFromPointCloud.h"
#include "lidar_viewer/geometry/functions/Utilities.h"
#include "lidar_viewer/geometry/functions/DownSample.h"
#include "lidar_viewer/geometry/functions/GetDepthImageToPointCloudProcessor.h"

namespace lidar_viewer::ui::display
{

using MapGlFloat3 = std::array<float , 3>;

template <typename CoordType>
using Box3D = lidar_viewer::geometry::types::Box<lidar_viewer::geometry::types::Point3D<CoordType>>;

bool displayOctreeFromPointCloud(const dev::CygLidarD1* lidar, lidar_viewer::ui::drawing::DrawCubeColorFloatArr drawCube
                                                                , lidar_viewer::ui::drawing::DrawPointColorFloatArr drawPoint)
{
    using geometry::types::PointCloud3D;
    using geometry::types::OctreeFromPointCloud;
    using geometry::functions::downSample;
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

    constexpr geometry::types::UintRange depthRange        = {0u, 3000u};

    constexpr geometry::types::DepthFrameAttributes depthFrameAttributes{dev::CygLidarD1::get3dFrameWindow(),
                                                        depthRange, 60.f, 32.5f};

    geometry::types::ScreenRangeGl glScreenRange{};

    auto conversionFunction = getDepthImageToPointCloudProcessor<DepthImage3D>(depthFrameAttributes, glScreenRange);
    PointCloud3D<float> pointCloudV;
    lidar->use3dPointCloudWithArgs(conversionFunction, pointCloudV);

    const auto pcDownSampled = downSample(pointCloudV, 0.13);

    for(auto point : pointCloudV)
    {
//        MapGlUByte3 pointColor{255, 255, 255};
//        glColor3ubv( pointColor.data() );
        drawPoint(point, {1.f, 1.f, 1.f});
    }
    if(pcDownSampled.size() <= 1)
    {
        return true;
    }
    OctreeFromPointCloud octree{pointCloudV, 32};

    for(const auto node : octree)
    {
        if(!node)
        {
            break ;
        }
        if(node->isDivided())
        {
            continue;
        }
        const auto bBox = node->getKey();
        MapGlFloat3 rgbValues{
                bBox.lo[2] < .5f ? 2 * bBox.lo[2] : 2 - 2 * bBox.lo[2], // g
                bBox.lo[2] < .5f ? 1 - 2 * bBox.lo[2] : depthRange.first, // r
                bBox.lo[2] < .5f ? depthRange.first : 2 * bBox.lo[2] - 1 // b
        };
//        glColor3fv( rgbValues.data() );
        drawCube(node->getKey(), rgbValues);
    }
    return true;
}

} // namespace lidar_viewer::ui::display