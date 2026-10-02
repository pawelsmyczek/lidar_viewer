#ifndef LIDAR_VIEWER_TRANSFORMPOINTCLOUD_H
#define LIDAR_VIEWER_TRANSFORMPOINTCLOUD_H
#include "lidar_viewer/geometry/types/PointCloud.h"
#include "lidar_viewer/geometry/types/Transform3D.h"
#include "lidar_viewer/geometry/types/Vector.h"

namespace lidar_viewer::geometry::functions
{

/// Applies a transform to every point of a cloud.
/// The cloud and the transform may use different coordinate types, e.g. a float cloud and a double
/// pose: every point is converted to the type of the transform, transformed there and converted
/// back, so the result has the coordinate type of the cloud.
/// @tparam C coordinate type of the cloud
/// @tparam T coordinate type of the transform
/// @param input the cloud, it is not modified
/// @param transform the transform applied to every point (rotation and translation)
/// @return new cloud with the points in the same order
template <typename C, typename T>
types::PointCloud3D<C> transformPointCloud(const types::PointCloud3D<C>& input, const types::Transform3D<T>& transform)
{
    types::PointCloud3D<C> result;
    result.reserve(result.size());
    for (const auto& point : input)
    {
        auto transformedVector = transform(toVector<T>(point));
        result.push_back(toPoint<C>(transformedVector));
    }
    return result;
}

} // namespace lidar_viewer::geometry::functions

#endif //LIDAR_VIEWER_TRANSFORMPOINTCLOUD_H
