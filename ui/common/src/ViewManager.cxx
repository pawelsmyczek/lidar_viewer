#include "lidar_viewer/ui/window/ViewManager.h"

namespace lidar_viewer::ui
{

void ViewManager::start(int *argc, char **argv)
{
    startImpl(argc, argv);
}

void ViewManager::stop()
{
    if(isStopped())
    {
        return ;
    }
    stopImpl();
}

bool ViewManager::isStopped()
{
    return stopped.load();
}

} // namespace lidar_viewer::ui