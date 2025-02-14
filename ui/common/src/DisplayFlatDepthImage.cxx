#include "lidar_viewer/ui/display/DisplayFlatDepthImage.h"

#include "lidar_viewer/geometry/types/Point.h"
#include "lidar_viewer/dev/CygLidarD1.h"
#include "lidar_viewer/geometry/types/Point.h"
#include "lidar_viewer/geometry/types/DepthFrameAttributes.h"
#include "lidar_viewer/geometry/types/ScreenRanges.h"
#include "lidar_viewer/geometry/functions/Utilities.h"

namespace lidar_viewer::ui
{
using MapGlFloat3 = std::array<float, 3>;
using MapGlUByte3 = std::array<uint8_t , 3>;

bool displayFlatDepthImage(const dev::CygLidarD1* lidar, const lidar_viewer::ui::drawing::DrawPointColorByteArr& drawPoint)
{
    using geometry::functions::valueToRGBByte;
    using geometry::functions::mapValue;

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

    constexpr std::pair<float, float> glFullScreenRangeX    {-1.f, 1.f};
    constexpr auto bUpperNormGlFullScreenRangeX = glFullScreenRangeX.second - glFullScreenRangeX.first;
    constexpr std::pair<float, float> glFullScreenRangeY    {1.f, -1.f};
    constexpr auto bUpperNormGlFullScreenRangeY = glFullScreenRangeY.second - glFullScreenRangeY.first;
    constexpr std::pair<float, float> glFullScreenRangeZ    {1.f, .0f};
    constexpr auto bUpperNormGlFullScreenRangeZ = glFullScreenRangeZ.second - glFullScreenRangeZ.first;

    constexpr std::pair<float, float> glRangeX              {.0f, static_cast<float>(depthFrameAttributes.frameResolution.first)};
    constexpr auto aUpperNormGlFullScreenRangeX = glRangeX.second - glRangeX.first;
    constexpr std::pair<float, float> glRangeY              {.0f, static_cast<float>(depthFrameAttributes.frameResolution.second)};
    constexpr auto aUpperNormGlFullScreenRangeY = glRangeY.second - glRangeY.first;
    constexpr std::pair<float, float> glRangeZ              {static_cast<float>(depthRange.first),
                                                                 static_cast<float>(depthRange.second)};
    constexpr auto aUpperNormGlFullScreenRangeZ = glRangeZ.second - glRangeZ.first;

    constexpr auto xUpperNormScalar = bUpperNormGlFullScreenRangeX / aUpperNormGlFullScreenRangeX;
    constexpr auto yUpperNormScalar = bUpperNormGlFullScreenRangeY / aUpperNormGlFullScreenRangeY;
    constexpr auto zUpperNormScalar = bUpperNormGlFullScreenRangeZ / aUpperNormGlFullScreenRangeZ;

    constexpr auto gScalar = ( depthRange.second - 1u ) * 255u;
    constexpr auto rScalar = ( ( depthRange.second / 2u ) - 1u ) * 255u;
    constexpr auto bScalar = ( ( depthRange.first ) - 1u ) * 255u;

    lidar->use3dPointCloud([&](const auto& pointCloud)
    {
    for ( auto y = 0u; y < depthFrameAttributes.frameResolution.second; ++y )
    {
        for ( auto x = 0u; x < depthFrameAttributes.frameResolution.first; ++x )
        {
            const auto elementOfFrame = pointCloud[y*depthFrameAttributes.frameResolution.first + x];
            // omit every point not fitting in range, even error frames
            if((elementOfFrame > depthRange.second)
               | (elementOfFrame < depthRange.first))
                continue ;

            MapGlUByte3 rgbValues{
                    valueToRGBByte<uint8_t>(gScalar, elementOfFrame),
                    valueToRGBByte<uint8_t>(rScalar, elementOfFrame),
                    valueToRGBByte<uint8_t>(bScalar, elementOfFrame)
            };

            geometry::types::Point3D<float> point{{
                  mapValue(glRangeX.first, glFullScreenRangeX.first,
                           xUpperNormScalar, static_cast<float>(x)),
                  mapValue(glRangeY.first, glFullScreenRangeY.first,
                           yUpperNormScalar, static_cast<float>(y)),
                  mapValue(glRangeZ.first, glFullScreenRangeZ.first,
                           zUpperNormScalar,static_cast<float>(elementOfFrame))
                                                  }};
            drawPoint(point, rgbValues);
        }
    }
    });
    return true;
}

} // namespace lidar_viewer::ui