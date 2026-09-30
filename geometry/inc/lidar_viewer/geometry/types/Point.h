#ifndef LIDAR_VIEWER_POINT_H
#define LIDAR_VIEWER_POINT_H

#include <array>

namespace lidar_viewer::geometry::types
{

/// Fixed-size point (or vector) with `Dimension` coordinates of type `CoordType`.
/// Thin wrapper around std::array offering element access and basic arithmetic.
/// @tparam CoordType type of a single coordinate, e.g. float
/// @tparam Dimension number of coordinates
template<typename CoordType, size_t Dimension>
struct Point
{
    /// number of coordinates of the point
    static constexpr auto Dim = Dimension;

    using value_type = CoordType;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    using reference = value_type&;
    using const_reference = const value_type&;


    /// constructs the point from an array of coordinates
    explicit Point(const std::array<CoordType, Dimension>& il)
    : x{il}
    {}

    /// unchecked coordinate access, `id` must be lower than Dim
    const_reference operator [] (size_t id) const
    {
        return x[id];
    }

    reference operator [] (size_t id)
    {
        return x[id];
    }

    /// coordinate access by value (unchecked, like operator[])
    value_type at(size_t id)
    {
        return x[id];
    }

    value_type at(size_t id) const
    {
        return x[id];
    }

    /// pointer to the first coordinate, coordinates are stored contiguously
    const_pointer data () const
    {
        return x.data();
    }

    /// coordinate-wise sum, returns a new point
    Point<CoordType, Dimension> operator + (const Point<CoordType, Dimension>& rhs)
    {
        Point<CoordType, Dimension> tmp;
        for (size_t i = 0; i < Dimension; ++i)
        {
            tmp[i] = x[i] + rhs[i];
        }
        return tmp;
    }

    /// coordinate-wise difference, returns a new point
    Point<CoordType, Dimension> operator - (const Point<CoordType, Dimension>& rhs)
    {
        Point<CoordType, Dimension> tmp;
        for (size_t i = 0; i < Dimension; ++i)
        {
            tmp[i] = x[i] - rhs[i];
        }
        return tmp;
    }

    /// adds `rhs` coordinate-wise to this point in place
    Point<CoordType, Dimension>& operator += (const Point<CoordType, Dimension>& rhs)
    {
        for (size_t i = 0; i < Dimension; ++i)
        {
            x[i] += rhs[i];
        }
        return *this;
    }

    /// divides every coordinate by `rhs` **in place** and returns a reference to this point,
    /// so, despite the operator, the receiver is modified (`rhs` must not be zero)
    Point<CoordType, Dimension>& operator / (size_t rhs)
    {
        for (size_t i = 0; i < Dimension; ++i)
        {
            x[i] /= rhs;
        }
        return *this;
    }

    /// default constructed point has uninitialised coordinates
    Point() = default;
    Point(const Point& ) = default;
    Point& operator = (const Point& ) = default;
    Point(Point&& ) = default;
    Point& operator = (Point&& ) = default;
private:
    std::array<CoordType, Dimension> x;
};

/// three dimensional point
template <typename CoordType>
using Point3D = Point<CoordType, 3>;

/// two dimensional point
template <typename CoordType>
using Point2D = Point<CoordType, 2>;


} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_POINT_H
