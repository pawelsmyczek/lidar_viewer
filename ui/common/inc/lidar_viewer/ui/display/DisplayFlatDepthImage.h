#ifndef LIDAR_VIEWER_DISPLAYFLATDEPTHIMAGE_H
#define LIDAR_VIEWER_DISPLAYFLATDEPTHIMAGE_H

#include "lidar_viewer/ui/drawing/DrawingFunctions.h"
#include "lidar_viewer/geometry/types/ScreenRanges.h"

namespace lidar_viewer::dev
{

class CygLidarD1;

} // namespace lidar_viewer::dev

namespace lidar_viewer::ui
{

bool displayFlatDepthImage(const dev::CygLidarD1* lidar, const lidar_viewer::geometry::types::ScreenRanges& screenRanges,
                           const lidar_viewer::ui::drawing::DrawPointColorByteArr& func);

} // namespace lidar_viewer::ui

#endif //LIDAR_VIEWER_DISPLAYFLATDEPTHIMAGE_H
