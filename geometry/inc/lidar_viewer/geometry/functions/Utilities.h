#ifndef LIDAR_VIEWER_UTILITIES_H
#define LIDAR_VIEWER_UTILITIES_H

#include "lidar_viewer/geometry/types/Box.h"
#include "lidar_viewer/geometry/types/PointCloud.h"

#include <cmath>
#include <functional>

namespace lidar_viewer::geometry::functions
{

/// Converts a value to a colour channel: `value * scalar`, rounded and limited to 255.
/// @note the result is cast to `ByteType` before it is limited
/// @tparam ByteType integral type of the channel, e.g. uint8_t
template<typename ByteType, typename ValueType>
ByteType valueToRGBByte(const unsigned int& scalar, const ValueType& value) noexcept
{
    auto result = static_cast<ByteType>(::roundf(static_cast<float>(value) * static_cast<float>(scalar)));
    // Clamp the result to the 0-255 range (since we're working with byte types)
    return result > 255u ? 255u : result < 0u ? 0u : static_cast<ByteType>(result);
}

/// Linear mapping of a value from range A to range B.
/// @param aFirst start of range A
/// @param bFirst start of range B
/// @param upperNormScalar ratio of the span of range B to the span of range A
/// @param inVal value from range A
/// @return the corresponding value in range B, i.e. `bFirst + (inVal - aFirst) * upperNormScalar`
template<typename tVal>
tVal mapValue(tVal aFirst, tVal bFirst, tVal upperNormScalar, tVal inVal) noexcept
{
    return bFirst + ((inVal - aFirst) * upperNormScalar);
}

/// Converts a distance and two angles to a cartesian point:
/// x = d * cos(rotationValueX) * sin(rotationValueY), y = d * sin(rotationValueX),
/// z = d * cos(rotationValueX) * cos(rotationValueY).
/// @param zDepth distance from the origin
/// @param rotationValueX elevation angle in radians (rotation around the horizontal axis)
/// @param rotationValueY azimuth angle in radians (rotation around the vertical axis)
template <typename T>
geometry::types::Point3D<T>
sphericalToEuclidean(const T zDepth, const T rotationValueX, const T rotationValueY)
{
    const auto cosY = std::cos(rotationValueY);
    const auto sinY = std::sin(rotationValueY);
    const auto cosX = std::cos(rotationValueX);
    const auto sinX = std::sin(rotationValueX);
    return geometry::types::Point3D<T>{
            {zDepth * cosX * sinY, zDepth * sinX, zDepth * cosX * cosY}};
}

/// Smallest axis aligned box containing all points of a 3D point cloud.
/// @note the cloud must not be empty
template<typename PointT>
types::Box<PointT> calculateBoundingBoxFromPointCloud(const types::PointCloud<PointT>& pointCloud)
{
    const auto [xMin, xMax] = std::minmax_element(pointCloud.begin(),pointCloud.end(),
                                                  [](const PointT& point1, const PointT& point2)
                                                  {
                                                      return point1.at(0) < point2.at(0);
                                                  });
    const auto [yMin, yMax] = std::minmax_element(pointCloud.begin(),pointCloud.end(),
                                                  [](const PointT& point1, const PointT& point2)
                                                  {
                                                      return point1.at(1) < point2.at(1);
                                                  });
    const auto [zMin, zMax] = std::minmax_element(pointCloud.begin(),pointCloud.end(),
                                                  [](const PointT & point1, const PointT& point2)
                                                  {
                                                      return point1.at(2) < point2.at(2);
                                                  });

    return {PointT{ {{(*xMax)[0],(*yMax)[1],(*zMax)[2]}} },PointT{ {{(*xMin)[0],(*yMin)[1],(*zMin)[2]}} }};
}

/// middle of the box along axis `i`
template <typename PointT>
PointT::value_type midOf(const types::Box<PointT>& box, size_t i)
{
    return (box.hi[i] + box.lo[i])/2;
}

/// Returns one of the eight octants of a 3D box.
/// @param opIndex octant number 0-7; bits 0, 1 and 2 select the lower half along x, y and z
///        respectively (0 is the upper octant on all axes, 7 is the lower octant on all axes)
template <typename PointT>
types::Box<PointT> subdivisionOfBounbdingBox(const types::Box<PointT>& box, size_t opIndex)
{
    using OperationsArray = std::array<std::function<types::Box<PointT> (types::Box<PointT> )>, 8>;
    static const auto opsArray = OperationsArray{{
        [](const types::Box<PointT>& box)
        {
            return types::Box<PointT>{box.hi, PointT{{midOf(box, 0), midOf(box, 1), midOf(box, 2)}}};
        },
        [](const types::Box<PointT>& box)
        {
            return types::Box<PointT>{PointT{{midOf(box, 0), box.hi[1], box.hi[2]}}, PointT{{box.lo[0], midOf(box, 1), midOf(box, 2)}}};
        },
        [](const types::Box<PointT>& box)
        {
            return types::Box<PointT>{PointT{{box.hi[0], midOf(box,1), box.hi[2]}}, PointT{{midOf(box, 0), box.lo[1], midOf(box, 2)}}};
        },
        [](const types::Box<PointT>& box)
        {
            return types::Box<PointT>{PointT{{midOf(box, 0), midOf(box,1), box.hi[2]}}, PointT{{box.lo[0], box.lo[1], midOf(box, 2)}}};
        },
        [](const types::Box<PointT>& box)
        {
            return types::Box<PointT>{PointT{{box.hi[0], box.hi[1], midOf(box,2)}}, PointT{{midOf(box,0), midOf(box,1), box.lo[2]}}};
        },
        [](const types::Box<PointT>& box)
        {
            return types::Box<PointT>{PointT{{midOf(box, 0), box.hi[1], midOf(box,2)}}, PointT{{box.lo[0], midOf(box,1), box.lo[2]}}};
        },
        [](const types::Box<PointT>& box)
        {
            return types::Box<PointT>{PointT{{box.hi[0], midOf(box,1), midOf(box,2)}}, PointT{{midOf(box, 0), box.lo[1], box.lo[2]}}};
        },
        [](const types::Box<PointT>& box)
        {
            auto hiTmp = box.hi;
            return types::Box<PointT>{(hiTmp + box.lo) / 2, box.lo};
        }
    }};
    return opsArray[opIndex](box);
}

} // namespace lidar_viewer::geometry::functions

#endif //LIDAR_VIEWER_UTILITIES_H
