#ifndef LIDAR_VIEWER_DRAWSTRING_H
#define LIDAR_VIEWER_DRAWSTRING_H

#include <array>
#include <string>

namespace lidar_viewer::ui::drawing
{

void drawStdStringIntPos(const std::string& string, const std::array<float, 3>& color, const int x, const int y);

void drawStdStringFloatPos(const std::string& string, const std::array<float, 3>& color, const float x, const float y);

void drawConstCharString(const char* string, const size_t size, const std::array<float, 3>& color, const size_t x, const size_t y);

} // namespace lidar_viewer::ui::drawing

#endif //LIDAR_VIEWER_DRAWSTRING_H
