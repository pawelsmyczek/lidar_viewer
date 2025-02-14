#ifndef LIDAR_VIEWER_POINTCLOUD_H
#define LIDAR_VIEWER_POINTCLOUD_H

#include "Point.h"

#include <vector>

namespace lidar_viewer::geometry::types
{

/// dynamically allocatable point cloud
template <typename PointType>
using PointCloud = std::vector<PointType>;

template <typename CoordType>
using PointCloud3D = PointCloud<Point3D<CoordType>>;

/// vector of indices pointing to an index in point cloud
using Indices = std::vector<size_t>;

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_POINTCLOUD_H
