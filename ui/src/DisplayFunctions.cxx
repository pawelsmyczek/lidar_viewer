#include "lidar_viewer/ui/DisplayFunctions.h"
#include "lidar_viewer/dev/CygLidarD1.h"

#include <GL/glew.h>
#include <GL/glut.h>

#include <array>
#include <iostream>
#include <cmath>

using MapGlFloat3 = std::array<GLfloat, 3>;
using MapGlUByte3 = std::array<GLubyte, 3>;

namespace
{

MapGlUByte3 valueRangeToRGBByte(unsigned int minv, unsigned int maxv, unsigned int value ) noexcept
{
    const auto ratio = static_cast<float>(value-minv) / static_cast<float>(maxv - minv);
    auto a = (1.f - ratio) / .25f;
    auto x = ::floorf(a);
    auto y = static_cast<GLubyte>(::floorf(255.f * (a - x)));

    switch(static_cast<uint8_t>(x))
    {
        case 0: return { 255u,                          y,                              0u};
        case 1: return { static_cast<GLubyte>(255u-y),  255u,                           0u};
        case 2: return { 0u,                            255u,                            y};
        case 3: return { 0u,                            static_cast<GLubyte>(255u-y),   255u};
        case 4: return { 0u,                            0u,                             255u};
    }
    return { 0u, 0u, 0u };
}

template<typename tVal>
tVal mapValue(std::pair<tVal,tVal> a, std::pair<tVal, tVal> b, tVal inVal) noexcept
{
    tVal inValNorm = inVal - a.first;
    tVal aUpperNorm = a.second - a.first;
    tVal normPosition = inValNorm / aUpperNorm;

    return b.first + (normPosition * ( b.second - b.first ) );
}

}

namespace lidar_viewer::ui
{

void lidar3D_Display(const dev::CygLidarD1* lidar)
{
    if(!lidar || !lidar->get3dFrame())
    {
        return ;
    }
    const auto& pointCloud = *lidar->get3dFrame();
    constexpr auto frameResolution = dev::CygLidarD1::getFrameWindow();

    static MapGlFloat3 positionMap{};
    constexpr auto minimumDistanceMm        = 51u;
    constexpr auto maximumDistanceMm        = 2000u;

    constexpr std::pair<GLfloat, GLfloat> glFullScreenRangeX    {-1.f, 1.f};
    constexpr std::pair<GLfloat, GLfloat> glFullScreenRangeY    {1.f, -1.f};
    constexpr std::pair<GLfloat, GLfloat> glFullScreenRangeZ    {1.f, .0f};
    constexpr std::pair<GLfloat, GLfloat> glRangeX              {0.f, static_cast<GLfloat>(frameResolution.first)};
    constexpr std::pair<GLfloat, GLfloat> glRangeY              {0.f, static_cast<GLfloat>(frameResolution.second)};
    constexpr std::pair<GLfloat, GLfloat> glRangeZ              {static_cast<GLfloat>(minimumDistanceMm),
                                                                 static_cast<GLfloat>(maximumDistanceMm)};

    if( pointCloud.size() != frameResolution.first * frameResolution.second )
    {
        throw std::runtime_error( "mismatched frameResolution and pointCloud.size(), they should match." );
    }

    glBegin(GL_POINTS);
    for ( auto y = 0u; y < frameResolution.second; ++y )
    {
        for ( auto x = 0u; x < frameResolution.first; ++x )
        {
            const auto frameId = y*frameResolution.first + x;
            if( pointCloud[frameId] == 4082u )
            {
                static const MapGlUByte3 rgbValuesForBrokenPoint { 66u, 135u, 245u };
                glColor3ubv( &rgbValuesForBrokenPoint[0] );
                positionMap =
                        {
                            mapValue<GLfloat>(glRangeX, glFullScreenRangeX, static_cast<GLfloat>(x)),
                            mapValue<GLfloat>(glRangeY, glFullScreenRangeY, static_cast<GLfloat>(y)),
                            mapValue<GLfloat>(glRangeZ, glFullScreenRangeZ, static_cast<GLfloat>(pointCloud[frameId]))
                        };
                glVertex3fv(&positionMap[0]);
                continue ;
            }

            if(pointCloud[frameId] > maximumDistanceMm
                        || pointCloud[frameId] < minimumDistanceMm)
                continue ;

            const auto rgbValues = valueRangeToRGBByte(minimumDistanceMm, maximumDistanceMm, pointCloud[frameId]);
            glColor3ubv( &rgbValues[0] );
            positionMap =
                    {
                        mapValue<GLfloat>(glRangeX, glFullScreenRangeX, static_cast<GLfloat>(x)),
                        mapValue<GLfloat>(glRangeY, glFullScreenRangeY, static_cast<GLfloat>(y)),
                        mapValue<GLfloat>(glRangeZ, glFullScreenRangeZ, static_cast<GLfloat>(pointCloud[frameId]))
                    };
            glVertex3fv(&positionMap[0]);
        }
    }
    glEnd();
}

}
