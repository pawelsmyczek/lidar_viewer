#ifndef LIDAR_VIEWER_GETDEPTHIMAGETOPOINTCLOUDPROCESSOR_H
#define LIDAR_VIEWER_GETDEPTHIMAGETOPOINTCLOUDPROCESSOR_H

#include "lidar_viewer/geometry/types/PointCloud.h"
#include "lidar_viewer/geometry/types/DepthFrameAttributes.h"
#include "lidar_viewer/geometry/types/ScreenRanges.h"
#include "Utilities.h"
#include <functional>

namespace lidar_viewer::geometry::functions
{

/// returns a function which will later process an input depth image to convert it to point cloud
template <typename FrameType>
std::function<void(const FrameType &, types::PointCloud3D<float>& )>
getDepthImageToPointCloudProcessor( const types::DepthFrameAttributes& depthAtributes, const types::ScreenRanges& screenRange)
{
    return [&depthAtributes, &screenRange](const FrameType& frame3d, types::PointCloud3D<float>& pointCloudV)
    {
        using lidar_viewer::geometry::functions::mapValue;
        using lidar_viewer::geometry::functions::sphericalToEuclidean;

        if(frame3d.empty())
        {
            return ;
        }
        const auto bUpperNormGlFullScreenRangeX = screenRange.fullRangeX().second - screenRange.fullRangeX().first;
        const auto bUpperNormGlFullScreenRangeY = screenRange.fullRangeY().second - screenRange.fullRangeY().first;
        const auto bUpperNormGlFullScreenRangeZ = screenRange.fullRangeZ().second - screenRange.fullRangeZ().first;

        const std::pair<float, float> glRangeX {.0f, static_cast<float>(depthAtributes.frameResolution.first)};
        const auto aUpperNormGlFullScreenRangeX = glRangeX.second - glRangeX.first;
        const std::pair<float, float> glRangeY {.0f, static_cast<float>(depthAtributes.frameResolution.second)};
        const auto aUpperNormGlFullScreenRangeY = glRangeY.second - glRangeY.first;
        const std::pair<float, float> glRangeZ {static_cast<float>(depthAtributes.depthRange.first),
                                                static_cast<float>(depthAtributes.depthRange.second)};
        const auto aUpperNormGlFullScreenRangeZ = glRangeZ.second - glRangeZ.first;

        const auto xUpperNormScalar = bUpperNormGlFullScreenRangeX / aUpperNormGlFullScreenRangeX;
        const auto yUpperNormScalar = bUpperNormGlFullScreenRangeY / aUpperNormGlFullScreenRangeY;
        const auto zUpperNormScalar = bUpperNormGlFullScreenRangeZ / aUpperNormGlFullScreenRangeZ;

        for (auto y = 0u; y < depthAtributes.frameResolution.second; ++y)
        {
            for (auto x = 0u; x < depthAtributes.frameResolution.first; ++x)
            {
                auto elementOfFrame = frame3d[y * depthAtributes.frameResolution.first + x];
                // omit every point not fitting in range, even error frames
                if ((elementOfFrame > depthAtributes.depthRange.second)
                    || (elementOfFrame < depthAtributes.depthRange.first))
                {
                    continue;

                }
                const auto xRotationPrecalc = mapValue(glRangeX.first, screenRange.fullRangeX().first,
                                                       xUpperNormScalar, static_cast<float>(x));
                const auto yRotationPrecalc = mapValue(glRangeY.first, screenRange.fullRangeY().first,
                                                       yUpperNormScalar, static_cast<float>(y));
                const auto zDepthPrecalc = mapValue(glRangeZ.first, screenRange.fullRangeZ().first,
                                                    zUpperNormScalar,static_cast<float>(elementOfFrame));
                const auto rotationValueX = yRotationPrecalc * depthAtributes.rotationY * M_PIf / 180.f;
                const auto rotationValueY = xRotationPrecalc * depthAtributes.rotationX * M_PIf / 180.f;
                auto rawPoint = sphericalToEuclidean(zDepthPrecalc, rotationValueX,
                                                     rotationValueY);

                pointCloudV.emplace_back(rawPoint);
            }
        }
    };
}

} // namespace lidar_viewer::geometry::functions

#endif //LIDAR_VIEWER_GETDEPTHIMAGETOPOINTCLOUDPROCESSOR_H
