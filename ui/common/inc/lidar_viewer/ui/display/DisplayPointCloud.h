#ifndef LIDAR_VIEWER_DISPLAYPOINTCLOUD_H
#define LIDAR_VIEWER_DISPLAYPOINTCLOUD_H

#include "lidar_viewer/ui/drawing/DrawingFunctions.h"

namespace lidar_viewer::dev
{

class CygLidarD1;

}

namespace lidar_viewer::ui
{

bool displayPointCloud3D(const dev::CygLidarD1* lidar, lidar_viewer::ui::drawing::DrawPointColorFloatArr&& ) noexcept;

bool displayPointCloud2D(const dev::CygLidarD1* lidar, lidar_viewer::ui::drawing::DrawPointColorFloatArr&& );


}
#endif // LIDAR_VIEWER_DISPLAYPOINTCLOUD_H
