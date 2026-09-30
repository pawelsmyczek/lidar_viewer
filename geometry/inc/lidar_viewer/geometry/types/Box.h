#ifndef LIDAR_VIEWER_BOX_H
#define LIDAR_VIEWER_BOX_H

#include "Point.h"

#include <cmath>

namespace lidar_viewer::geometry::types
{

/// Axis aligned bounding box described by two opposite corners.
/// @tparam PointT point type, see Point; must expose `Dim`, `value_type` and operator[]
template <typename PointT>
struct Box
{
    using CoordType = PointT::value_type;
    /// @param hi_ corner with the highest coordinate on every axis
    /// @param lo_ corner with the lowest coordinate on every axis
    Box(const PointT& hi_, const PointT& lo_ )
    : hi{hi_}
    , lo{lo_}
    { }

    Box(PointT&& hi_, PointT&& lo_ )
            : hi{std::move(hi_)}
            , lo{std::move(lo_)}
    { }

    /// Euclidean distance from `point` to the box, zero when the point is inside or on the border
    CoordType distance(const PointT& point) const
    {
        CoordType dd{};
        for (size_t i = 0u; i < PointT::Dim; ++i)
        {
            if(point[i] < lo[i])
            {
                dd += std::pow(point[i] - lo[i], 2);
            }
            if(point[i] > hi[i])
            {
                dd += std::pow(point[i] - hi[i], 2);
            }
        }
        return std::sqrt(dd);
    }

    /// true if `point` lies inside the box (borders included)
    bool contains(const PointT& point) const
    {
        return distance(point) == CoordType{0};
    }

// private:
    PointT hi; ///< upper corner
    PointT lo; ///< lower corner
};

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_BOX_H
