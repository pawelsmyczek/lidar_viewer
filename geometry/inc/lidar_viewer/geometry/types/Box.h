#ifndef LIDAR_VIEWER_BOX_H
#define LIDAR_VIEWER_BOX_H

#include "Point.h"

#include <cmath>

namespace lidar_viewer::geometry::types
{

template <typename PointT>
struct Box
{
    using CoordType = PointT::value_type;
    Box(const PointT& hi_, const PointT& lo_ )
    : hi{hi_}
    , lo{lo_}
    { }

    Box(PointT&& hi_, PointT&& lo_ )
            : hi{std::move(hi_)}
            , lo{std::move(lo_)}
    { }

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

    bool contains(const PointT& point) const
    {
        return distance(point) == CoordType{0};
    }

// private:
    PointT hi;
    PointT lo;
};

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_BOX_H
