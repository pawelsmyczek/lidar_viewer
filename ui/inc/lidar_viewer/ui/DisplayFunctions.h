#ifndef LIDAR_VIEWER_DISPLAYFUNCTIONS_H
#define LIDAR_VIEWER_DISPLAYFUNCTIONS_H

namespace lidar_viewer::dev
{

class CygLidarD1;

}

namespace lidar_viewer::ui
{

void lidar3D_Display(const dev::CygLidarD1* lidar);

}
#endif // LIDAR_VIEWER_DISPLAYFUNCTIONS_H
