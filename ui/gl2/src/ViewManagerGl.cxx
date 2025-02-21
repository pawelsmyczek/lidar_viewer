#include "lidar_viewer/ui/window/gl2/ViewManagerGl.h"

#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

namespace lidar_viewer::ui
{

ViewManagerGl::ViewManagerGl(ViewManagerGl::Config config)
: ViewManager()
, configuration{config}
{
}

void ViewManagerGl::startImpl(int *argc, char **argv)
{
    glutInit(argc, argv);
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION); // continue execution ofter window close to cleanup properly
    glutInitWindowSize(static_cast<int>(configuration.w),
                       static_cast<int>(configuration.h));
    windowId = glutCreateWindow("LidarViewer");
    glutSetWindow(windowId);
}

void ViewManagerGl::stopImpl()
{
    stopped = true;
    auto id = glutGetWindow();
    if(id != windowId)
    {
        std::cout << "window already closed\n";
        return ;
    }

    std::cout << "trying to destroy window : " << windowId << '\n';
    glutDestroyWindow(windowId);
}

ViewManagerGl::~ViewManagerGl()
{
    try
    {
        stop();
    }
    catch ( ... )
    {

    }
}


} // namespace lidar_viewer::ui
