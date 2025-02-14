#include "lidar_viewer/ui/DisplayFlatDepthImage.h"

#include "lidar_viewer/dev/CygLidarD1.h"
#include "lidar_viewer/ui/DrawPoint.h"
#include "lidar_viewer/geometry/types/Point.h"
#include "lidar_viewer/geometry/types/DepthFrameAttributes.h"
#include "lidar_viewer/geometry/types/ScreenRanges.h"
#include "lidar_viewer/geometry/functions/Utilities.h"

#include <GL/glew.h>


namespace lidar_viewer::ui
{
using MapGlFloat3 = std::array<GLfloat, 3>;
using MapGlUByte3 = std::array<GLubyte , 3>;

bool displayFlatDepthImage(const dev::CygLidarD1* lidar)
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

    constexpr std::pair<GLfloat, GLfloat> glFullScreenRangeX    {-1.f, 1.f};
    constexpr auto bUpperNormGlFullScreenRangeX = glFullScreenRangeX.second - glFullScreenRangeX.first;
    constexpr std::pair<GLfloat, GLfloat> glFullScreenRangeY    {1.f, -1.f};
    constexpr auto bUpperNormGlFullScreenRangeY = glFullScreenRangeY.second - glFullScreenRangeY.first;
    constexpr std::pair<GLfloat, GLfloat> glFullScreenRangeZ    {1.f, .0f};
    constexpr auto bUpperNormGlFullScreenRangeZ = glFullScreenRangeZ.second - glFullScreenRangeZ.first;

    constexpr std::pair<GLfloat, GLfloat> glRangeX              {.0f, static_cast<GLfloat>(depthFrameAttributes.frameResolution.first)};
    constexpr auto aUpperNormGlFullScreenRangeX = glRangeX.second - glRangeX.first;
    constexpr std::pair<GLfloat, GLfloat> glRangeY              {.0f, static_cast<GLfloat>(depthFrameAttributes.frameResolution.second)};
    constexpr auto aUpperNormGlFullScreenRangeY = glRangeY.second - glRangeY.first;
    constexpr std::pair<GLfloat, GLfloat> glRangeZ              {static_cast<GLfloat>(depthRange.first),
                                                                 static_cast<GLfloat>(depthRange.second)};
    constexpr auto aUpperNormGlFullScreenRangeZ = glRangeZ.second - glRangeZ.first;

    constexpr auto xUpperNormScalar = bUpperNormGlFullScreenRangeX / aUpperNormGlFullScreenRangeX;
    constexpr auto yUpperNormScalar = bUpperNormGlFullScreenRangeY / aUpperNormGlFullScreenRangeY;
    constexpr auto zUpperNormScalar = bUpperNormGlFullScreenRangeZ / aUpperNormGlFullScreenRangeZ;

    constexpr auto gScalar = ( depthRange.second - 1u ) * 255u;
    constexpr auto rScalar = ( ( depthRange.second / 2u ) - 1u ) * 255u;
    constexpr auto bScalar = ( ( depthRange.first ) - 1u ) * 255u;

    lidar->use3dPointCloud([&](const auto& pointCloud)
   {
    glBegin(GL_POINTS);
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
                    valueToRGBByte<GLubyte>(gScalar, elementOfFrame),
                    valueToRGBByte<GLubyte>(rScalar, elementOfFrame),
                    valueToRGBByte<GLubyte>(bScalar, elementOfFrame)
            };

            MapGlFloat3 positionMap{
                    mapValue(glRangeX.first, glFullScreenRangeX.first, xUpperNormScalar, static_cast<GLfloat>(x)),
                    mapValue(glRangeY.first, glFullScreenRangeY.first, yUpperNormScalar, static_cast<GLfloat>(y)),
                    mapValue(glRangeZ.first, glFullScreenRangeZ.first, zUpperNormScalar, static_cast<GLfloat>(elementOfFrame))
            };
            glVertex3fv( positionMap.data() );
            glColor3ubv( rgbValues.data() );
        }
    }
    glEnd();
});
    return true;
}

} // namespace lidar_viewer::ui