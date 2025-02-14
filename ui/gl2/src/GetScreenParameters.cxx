#include "lidar_viewer/ui/window/gl2/GetScreenParameters.h"

#include <GL/glut.h>

namespace lidar_viewer::ui
{

void getScreenParameters(ScreenParameters<int>& screen)
{
    static_assert(sizeof(decltype(screen)) == 4*sizeof(int), "Screen params should contain 4 parameters");
    glGetIntegerv( GL_VIEWPORT, reinterpret_cast<GLint*>(&screen) );
}

} // namespace lidar_viewer::ui