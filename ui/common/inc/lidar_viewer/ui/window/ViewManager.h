#ifndef LIDAR_VIEWER_VIEWMANAGER_H
#define LIDAR_VIEWER_VIEWMANAGER_H

#include "lidar_viewer/ui/display/DisplayManager.h"

#include <atomic>
#include <functional>
#include <iostream>
#include <mutex>
#include <memory>
#include <vector>
#include <thread>
#include <sstream>

namespace lidar_viewer::ui
{

struct ViewManager
{

    ViewManager()
            : stopped{}
    {
    }
    virtual ~ViewManager() = default;

    /// starts displaying, shall be used as a main thread loop
    void start(int* argc, char** argv);

    /// closes the window, shall be used only in signal handlers etc
    void stop();

    bool isStopped();
private:
    virtual void startImpl(int *argc, char **argv) = 0;
    virtual void stopImpl() = 0;

protected:
    std::atomic<bool> stopped;
};

} // namespace lidar_viewer::ui

#endif //LIDAR_VIEWER_VIEWMANAGER_H
