#ifndef LIDAR_VIEWER_VIEWMANAGERGL_H
#define LIDAR_VIEWER_VIEWMANAGERGL_H

#include "lidar_viewer/ui/window/ViewManager.h"
#include "lidar_viewer/ui/display/gl2/DisplayManagerGl.h"

#include <chrono>
#include <memory>

namespace lidar_viewer::ui
{

struct ViewManagerGl
        : public ViewManager
{
    struct Config
    {
        unsigned int x;
        unsigned int y;
        unsigned int w;
        unsigned int h;
    };

    ViewManagerGl(Config );
    ~ViewManagerGl() override;

    void startImpl(int *argc, char **argv) override;
    void stopImpl() override;

private:
    Config configuration;
    std::atomic<int> rotx;
    std::atomic<int> roty;
    float windowScale;
    int windowId;
};

} // namespace lidar_viewer::ui

#endif //LIDAR_VIEWER_VIEWMANAGERGL_H
