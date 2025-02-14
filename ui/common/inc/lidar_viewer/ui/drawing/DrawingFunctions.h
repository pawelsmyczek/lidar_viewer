#ifndef LIDAR_VIEWER_DRAWINGFUNCTIONS_H
#define LIDAR_VIEWER_DRAWINGFUNCTIONS_H

#include "lidar_viewer/geometry/types/Box.h"
#include "lidar_viewer/geometry/types/Point.h"

#include <array>
#include <functional>
#include <string>

namespace lidar_viewer::ui::drawing
{

template <typename ColorProvider>
using DrawCube = std::function<void(const geometry::types::Box<geometry::types::Point3D<float>>&,
                                                                    const ColorProvider&)>;

template <typename ColorProvider>
using DrawPoint = std::function<void(const geometry::types::Point3D<float>&,
                                     const ColorProvider&)>;

template <typename ColorProvider>
using DrawStdString = std::function<void(const std::string&,
                                         const ColorProvider&, const float, const float)>;

template <typename ColorProvider>
using DrawConstCharString = std::function<void(const char*,
                                               const ColorProvider&, float, float)>;

using DrawCubeColorFloatArr = DrawCube<std::array<float, 3>>;

using DrawPointColorFloatArr = DrawPoint<std::array<float, 3>>;

using DrawPointColorByteArr = DrawPoint<std::array<uint8_t, 3>>;

using DrawStdStringColorFloatArr = DrawStdString<std::array<float, 3>>;

}// namespace lidar_viewer::ui::drawing

#endif //LIDAR_VIEWER_DRAWINGFUNCTIONS_H
