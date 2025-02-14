#include "lidar_viewer/ui/DisplayFunctions.h"
#include "lidar_viewer/dev/CygLidarD1.h"

#include <GL/glew.h>
#include <GL/glut.h>

#include <array>
#include <cmath>

using MapGlFloat3 = std::array<GLfloat, 3>;
using MapGlUByte3 = std::array<GLubyte, 3>;
using EC3D = lidar_viewer::dev::CygLidarD1::ErrorCodes3D;

namespace
{

template <typename ValueType>
GLubyte valueToRGBByte(unsigned int scalar, ValueType value ) noexcept
{
    return static_cast<GLubyte>(::roundf(static_cast<float>(value) * static_cast<float>(scalar) ));
}

template<typename tVal>
tVal mapValue(tVal aFirst, tVal bFirst, tVal upperNormScalar, tVal inVal) noexcept
{
    return bFirst + ( (inVal - aFirst) * upperNormScalar );
}

[[maybe_unused]] bool isBrokenPoint(EC3D value)
{
    return value >= EC3D::LowAmplitude && value <= EC3D::Saturation;
}

[[maybe_unused]] constexpr MapGlUByte3 rgbValuesForBrokenPoint(EC3D errorValue)
{
    switch (errorValue)
    {
        case EC3D::LowAmplitude:
            return { 255u, 255u, 255u };
        case EC3D::AdcOverflow:
            return { 66u, 135u, 245u };
        case EC3D::Saturation:
            return { 255u, 0u, 255u };
    }
}

}

namespace lidar_viewer::ui
{

bool lidar3D_Display(const dev::CygLidarD1* lidar) noexcept
{
    if(!lidar)
    {
        return false;
    }
    if(lidar->failedToRead())
    {
        return false;
    }
    constexpr auto frameResolution = dev::CygLidarD1::get3dFrameWindow();

    constexpr auto minimumDistanceMm        = 51u;
    constexpr auto maximumDistanceMm        = 2000u;

    constexpr auto gScalar = ( maximumDistanceMm - 1u ) * 255u;
    constexpr auto rScalar = ( ( maximumDistanceMm / 2u ) - 1u ) * 255u;
    constexpr auto bScalar = ( ( minimumDistanceMm ) - 1u ) * 255u;

    constexpr std::pair<GLfloat, GLfloat> glFullScreenRangeX    {-1.f, 1.f};
    constexpr auto bUpperNormGlFullScreenRangeX = glFullScreenRangeX.second - glFullScreenRangeX.first;
    constexpr std::pair<GLfloat, GLfloat> glFullScreenRangeY    {1.f, -1.f};
    constexpr auto bUpperNormGlFullScreenRangeY = glFullScreenRangeY.second - glFullScreenRangeY.first;
    constexpr std::pair<GLfloat, GLfloat> glFullScreenRangeZ    {1.f, .0f};
    constexpr auto bUpperNormGlFullScreenRangeZ = glFullScreenRangeZ.second - glFullScreenRangeZ.first;

    constexpr std::pair<GLfloat, GLfloat> glRangeX              {.0f, static_cast<GLfloat>(frameResolution.first)};
    constexpr auto aUpperNormGlFullScreenRangeX = glRangeX.second - glRangeX.first;
    constexpr std::pair<GLfloat, GLfloat> glRangeY              {.0f, static_cast<GLfloat>(frameResolution.second)};
    constexpr auto aUpperNormGlFullScreenRangeY = glRangeY.second - glRangeY.first;
    constexpr std::pair<GLfloat, GLfloat> glRangeZ              {static_cast<GLfloat>(minimumDistanceMm),
                                                          static_cast<GLfloat>(maximumDistanceMm)};
    constexpr auto aUpperNormGlFullScreenRangeZ = glRangeZ.second - glRangeZ.first;

    constexpr auto xUpperNormScalar = bUpperNormGlFullScreenRangeX / aUpperNormGlFullScreenRangeX;
    constexpr auto yUpperNormScalar = bUpperNormGlFullScreenRangeY / aUpperNormGlFullScreenRangeY;
    constexpr auto zUpperNormScalar = bUpperNormGlFullScreenRangeZ / aUpperNormGlFullScreenRangeZ;

    lidar->use3dPointCloud([&frameResolution](const auto& pointCloud)
    {
        glBegin(GL_POINTS);
        for ( auto y = 0u; y < frameResolution.second; ++y )
        {
            for ( auto x = 0u; x < frameResolution.first; ++x )
            {
                auto elementOfFrame = pointCloud[y*frameResolution.first + x];
                // omit every point not fitting in range, even error frames
                if((elementOfFrame > maximumDistanceMm)
                   | (elementOfFrame < minimumDistanceMm))
                    continue ;

                MapGlUByte3 rgbValues{
                        valueToRGBByte(gScalar, elementOfFrame),
                        valueToRGBByte(rScalar, elementOfFrame),
                        valueToRGBByte(bScalar, elementOfFrame)
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

bool lidar2D_Display(const dev::CygLidarD1* lidar)
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
