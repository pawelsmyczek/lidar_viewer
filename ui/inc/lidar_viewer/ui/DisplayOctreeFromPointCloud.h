#ifndef LIDAR_VIEWER_DISPLAYOCTREEFROMPOINTCLOUD_H
#define LIDAR_VIEWER_DISPLAYOCTREEFROMPOINTCLOUD_H

namespace lidar_viewer::dev
{

class CygLidarD1;

}

namespace lidar_viewer::ui
{

bool displayOctreeFromPointCloud(const dev::CygLidarD1* lidar);

}

#endif //LIDAR_VIEWER_DISPLAYOCTREEFROMPOINTCLOUD_H
