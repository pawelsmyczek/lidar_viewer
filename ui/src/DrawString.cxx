#include "lidar_viewer/ui/DrawString.h"

#include <GL/glut.h>

namespace lidar_viewer::ui
{

void drawString(const std::string& string, const int x, const int y)
{
    glRasterPos2i(x, y);
    for (const auto& character : string)
    {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, character);
    }
}

void drawString(const std::string& string, const float x, const float y)
{
    glRasterPos2f(x, y);
    for (const auto& character : string)
    {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, character);
    }
}

void drawString(const char* string, const size_t size, const size_t x, const size_t y)
{
    glRasterPos2i(x, y);
    if(!string)
    {
        return ;
    }
    for (auto i = 0u; i < size; ++i)
    {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, string[i]);
    }
}

} // namespace lidar_viewer::ui
