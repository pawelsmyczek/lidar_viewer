#include "lidar_viewer/ui/DisplayStatistics.h"
#include "lidar_viewer/ui/DrawString.h"

#include <chrono>
#include <sstream>

#include <GL/gl.h>

namespace lidar_viewer::ui
{

static std::chrono::high_resolution_clock::time_point previousInterval{};

bool displayStatistics()
{
    struct ViewPort
    {
        GLint x, y, w, h;
    } screen;
    glGetIntegerv( GL_VIEWPORT, reinterpret_cast<GLint*>(&screen) );

    using namespace std::string_literals;
    using namespace std::chrono_literals;

    std::stringstream sstreamFrameTime;
    std::stringstream sstreamFramePerSec;

    const auto currentInterval = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double, std::chrono::seconds::period> timeSpentOnRender = currentInterval - previousInterval;
    const auto framesPerSecond = std::chrono::duration<double, std::chrono::seconds::period>(1s).count() / timeSpentOnRender.count();
//    if(framesPerSecond > 150)
//    {
//        return true;
//    }
    previousInterval = std::chrono::high_resolution_clock::now();

    sstreamFrameTime   << "Time    : "s << timeSpentOnRender.count() << "s";
    sstreamFramePerSec << "FPS     : "s << static_cast<int>(framesPerSecond);

    const float xFrameTime =  -static_cast<float>(screen.w) / screen.w;
    const float yFrameTime =  -static_cast<float>(screen.h-23) / screen.h;
    const float xFramePerSecond =  -static_cast<float>(screen.w) / screen.w;
    const float yFramePerSecond =  -static_cast<float>(screen.h) / screen.h;

    glColor3ub(255, 255, 255);
    drawString(sstreamFrameTime.str(), xFrameTime, yFrameTime);
    drawString(sstreamFramePerSec.str(), xFramePerSecond, yFramePerSecond);

    return true;
}

} // namespace lidar_viewer::ui