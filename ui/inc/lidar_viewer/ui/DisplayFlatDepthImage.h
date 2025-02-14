#ifndef LIDAR_VIEWER_DISPLAYFLATDEPTHIMAGE_H
#define LIDAR_VIEWER_DISPLAYFLATDEPTHIMAGE_H

namespace lidar_viewer::dev
{

class CygLidarD1;

} // namespace lidar_viewer::dev

namespace lidar_viewer::ui
{

bool displayFlatDepthImage(const dev::CygLidarD1* lidar);

} // namespace lidar_viewer::ui

#endif //LIDAR_VIEWER_DISPLAYFLATDEPTHIMAGE_H
