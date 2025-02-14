#ifndef LIDAR_VIEWER_DRAWSTRING_H
#define LIDAR_VIEWER_DRAWSTRING_H

#include <string>

namespace lidar_viewer::ui
{

void drawString(const std::string& string, const int x, const int y);

void drawString(const std::string& string, const float x, const float y);

void drawString(const char* string, const size_t size, const size_t x, const size_t y);

} // namespace lidar_viewer::ui

#endif //LIDAR_VIEWER_DRAWSTRING_H
