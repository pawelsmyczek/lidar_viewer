#ifndef LIDAR_VIEWER_DISPLAYOCTREEFROMPOINTCLOUD_H
#define LIDAR_VIEWER_DISPLAYOCTREEFROMPOINTCLOUD_H

#include "lidar_viewer/ui/drawing/DrawingFunctions.h"

namespace lidar_viewer::dev
{

class CygLidarD1;

}

namespace lidar_viewer::ui::display
{

bool displayOctreeFromPointCloud(const dev::CygLidarD1* lidar, lidar_viewer::ui::drawing::DrawCubeColorFloatArr
        , lidar_viewer::ui::drawing::DrawPointColorFloatArr );

} // namespace lidar_viewer::ui::display

#endif //LIDAR_VIEWER_DISPLAYOCTREEFROMPOINTCLOUD_H
