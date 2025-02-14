#include "lidar_viewer/ui/drawing/gl2/DrawString.h"

#include <GL/glut.h>

namespace lidar_viewer::ui::drawing
{

void drawStdStringIntPos(const std::string& string, const std::array<float, 3>& color, const int x, const int y)
{
    glColor3fv(color.data());
    glRasterPos2i(x, y);
    for (const auto& character : string)
    {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, character);
    }
}

void drawStdStringFloatPos(const std::string& string, const std::array<float, 3>& color, const float x, const float y)
{
    glColor3fv(color.data());
    glRasterPos2f(x, y);
    for (const auto& character : string)
    {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, character);
    }
}

void drawConstCharString(const char* string, const size_t size, const std::array<float, 3>& color, const size_t x, const size_t y)
{
    glColor3fv(color.data());
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

} // namespace lidar_viewer::ui::drawing

