#ifndef LIDAR_VIEWER_DISPLAYPOINTCLOUD_H
#define LIDAR_VIEWER_DISPLAYPOINTCLOUD_H

namespace lidar_viewer::dev
{

class CygLidarD1;

}

namespace lidar_viewer::ui
{

bool displayPointCloud3D(const dev::CygLidarD1* lidar) noexcept;

bool displayPointCloud2D(const dev::CygLidarD1* lidar);


}
#endif // LIDAR_VIEWER_DISPLAYPOINTCLOUD_H
