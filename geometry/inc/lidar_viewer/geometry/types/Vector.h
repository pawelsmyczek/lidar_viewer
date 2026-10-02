#ifndef LIDAR_VIEWER_VECTOR_H
#define LIDAR_VIEWER_VECTOR_H

#include "Point.h"

#include <concepts>
#include <cstddef>

namespace lidar_viewer::geometry::types
{

/// with a Point class we can easily create a vector of a static size
/// this alias provides the distinction between those 2, as they're used
/// in little different contexts
template <typename CoordType, size_t Dimension>
using Vector = Point<CoordType, Dimension>;

/// three dimensional vector
template <typename CoordType>
using Vector3D = Point3D<CoordType>;

/// two dimensional vector
template <typename CoordType>
using Vector2D = Point2D<CoordType>;

/// `V` is exactly a Vector (a Point) with `D` coordinates of type `T`, no conversions allowed
/// @tparam V the type to check
/// @tparam T the required coordinate type
/// @tparam D the required number of coordinates
template <typename V, typename T, size_t D>
concept VectorOf = std::same_as<V, Vector<T, D>>;

/// The position `p` seen as a vector from the origin, with the coordinates converted to `T`.
/// Vector3D is an alias of Point3D, so this changes the name of the meaning and the coordinate
/// type, not the representation.
/// @tparam T coordinate type of the result, to be given explicitly: `toVector<double>(p)`
template <typename T, typename C>
constexpr Vector3D<T> toVector(const Point3D<C>& p)
{
    return Vector3D<T>{p};
}

/// The vector `v` seen as a position, with the coordinates converted to `C`.
/// @tparam C coordinate type of the result, to be given explicitly: `toPoint<float>(v)`
template <typename C, typename T>
constexpr Point3D<C> toPoint(const Vector3D<T>& v)
{
    return Point3D<C>{v};
}

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_VECTOR_H
