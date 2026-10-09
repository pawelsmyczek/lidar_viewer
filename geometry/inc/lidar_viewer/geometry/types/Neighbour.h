#ifndef LIDAR_VIEWER_NEIGHBOUR_H
#define LIDAR_VIEWER_NEIGHBOUR_H

#include <cstddef>

namespace lidar_viewer::geometry::types
{

/// Result of a nearest neighbour query: a point of the searched cloud and its distance to the query.
/// @tparam CoordType type of the distance, the coordinate type of the cloud
template <typename CoordType>
struct Neighbour
{
    size_t index;         ///< index of the point in the searched point cloud
    CoordType distance;   ///< Euclidean distance from the query to that point

    bool operator == (const Neighbour&) const = default;
};

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_NEIGHBOUR_H
