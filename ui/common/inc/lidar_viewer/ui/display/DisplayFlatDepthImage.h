#ifndef LIDAR_VIEWER_DISPLAYFLATDEPTHIMAGE_H
#define LIDAR_VIEWER_DISPLAYFLATDEPTHIMAGE_H

#include "lidar_viewer/ui/drawing/DrawingFunctions.h"

namespace lidar_viewer::dev
{

class CygLidarD1;

} // namespace lidar_viewer::dev

namespace lidar_viewer::ui
{

bool displayFlatDepthImage(const dev::CygLidarD1* lidar, const lidar_viewer::ui::drawing::DrawPointColorByteArr& );

} // namespace lidar_viewer::ui

#endif //LIDAR_VIEWER_DISPLAYFLATDEPTHIMAGE_H
