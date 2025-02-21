#ifndef LIDAR_VIEWER_UTILITIES_H
#define LIDAR_VIEWER_UTILITIES_H

#include "lidar_viewer/geometry/types/Box.h"
#include "lidar_viewer/geometry/types/PointCloud.h"

#include <cmath>
#include <functional>

namespace lidar_viewer::geometry::functions
{

template<typename ByteType, typename ValueType>
ByteType valueToRGBByte(const unsigned int& scalar, const ValueType& value) noexcept
{
    auto result = static_cast<ByteType>(::roundf(static_cast<float>(value) * static_cast<float>(scalar)));
    // Clamp the result to the 0-255 range (since we're working with byte types)
    return result > 255u ? 255u : result < 0u ? 0u : static_cast<ByteType>(result);
}

template<typename tVal>
tVal mapValue(tVal aFirst, tVal bFirst, tVal upperNormScalar, tVal inVal) noexcept
{
    return bFirst + ((inVal - aFirst) * upperNormScalar);
}

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

template <typename PointT>
PointT::value_type midOf(const types::Box<PointT>& box, size_t i)
{
    return (box.hi[i] + box.lo[i])/2;
}

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
