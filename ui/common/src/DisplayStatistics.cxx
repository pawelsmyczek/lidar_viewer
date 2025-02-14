#include "lidar_viewer/ui/display/DisplayStatistics.h"

#include <chrono>
#include <sstream>

#include <GL/gl.h>

namespace lidar_viewer::ui::display
{

static std::chrono::high_resolution_clock::time_point previousInterval{};

bool displayStatistics(const lidar_viewer::ui::drawing::DrawStdStringColorFloatArr& drawString,
                       const lidar_viewer::ui::ScreenParametersGetterInt& getScreenParameters)
{
    using namespace std::string_literals;
    using namespace std::chrono_literals;

    std::stringstream sstreamFrameTime;
    std::stringstream sstreamFramePerSec;

    ScreenParameters<int> screen{};
    getScreenParameters(screen);
    const auto currentInterval = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double, std::chrono::seconds::period> timeSpentOnRender = currentInterval - previousInterval;
    const auto framesPerSecond = std::chrono::duration<double, std::chrono::seconds::period>(1s).count() / timeSpentOnRender.count();

    previousInterval = std::chrono::high_resolution_clock::now();

    sstreamFrameTime   << "Time    : "s << timeSpentOnRender.count() << "s";
    sstreamFramePerSec << "FPS     : "s << static_cast<int>(framesPerSecond);

    const float xFrameTime =  -static_cast<float>(screen.w) / screen.w;
    const float yFrameTime =  -static_cast<float>(screen.h-23) / screen.h;

    const float xFramePerSecond =  -static_cast<float>(screen.w) / screen.w;
    const float yFramePerSecond =  -static_cast<float>(screen.h) / screen.h;

    drawString(sstreamFrameTime.str(), {1.f, 1.f, 1.f}, xFrameTime, yFrameTime);
    drawString(sstreamFramePerSec.str(), {1.f, 1.f, 1.f}, xFramePerSecond, yFramePerSecond);

    return true;
}

} // namespace lidar_viewer::ui::display