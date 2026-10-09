#ifndef LIDAR_VIEWER_BOX_H
#define LIDAR_VIEWER_BOX_H

#include "lidar_viewer/linalg/Point.h"

#include <cmath>

namespace lidar_viewer::geometry::types
{

/// Axis aligned bounding box described by two opposite corners.
/// @tparam PointT point type, see linalg::Point; must expose `Dim`, `value_type` and operator[]
template <typename PointT>
struct Box
{
    using CoordType = PointT::value_type;
    /// @param hi_ corner with the highest coordinate on every axis
    /// @param lo_ corner with the lowest coordinate on every axis
    constexpr Box(const PointT& hi_, const PointT& lo_ )
    : hi{hi_}
    , lo{lo_}
    { }

    constexpr Box(PointT&& hi_, PointT&& lo_ )
            : hi{std::move(hi_)}
            , lo{std::move(lo_)}
    { }

    /// Squared Euclidean distance from `point` to the box, zero when the point is inside or on the
    /// border. Enough for comparing distances, it avoids the square root.
    constexpr CoordType distanceSquared(const PointT& point) const
    {
        CoordType dd{};
        for (size_t i = 0u; i < PointT::Dim; ++i)
        {
            if(point[i] < lo[i])
            {
                dd += (point[i] - lo[i]) * (point[i] - lo[i]);
            }
            if(point[i] > hi[i])
            {
                dd += (point[i] - hi[i]) * (point[i] - hi[i]);
            }
        }
        return dd;
    }

    /// Euclidean distance from `point` to the box, zero when the point is inside or on the border
    constexpr CoordType distance(const PointT& point) const
    {
        return std::sqrt(distanceSquared(point));
    }

    /// true if `point` lies inside the box (borders included)
    constexpr bool contains(const PointT& point) const
    {
        return distanceSquared(point) == CoordType{0};
    }

// private:
    PointT hi; ///< upper corner
    PointT lo; ///< lower corner
};

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_BOX_H
