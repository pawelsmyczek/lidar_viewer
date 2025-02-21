#ifndef LIDAR_VIEWER_POINT_H
#define LIDAR_VIEWER_POINT_H

#include <array>

namespace lidar_viewer::geometry::types
{

template<typename CoordType, size_t Dimension>
struct Point
{
    static constexpr auto Dim = Dimension;

    using value_type = CoordType;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    using reference = value_type&;
    using const_reference = const value_type&;


    explicit Point(const std::array<CoordType, Dimension>& il)
    : x{il}
    {}

    const_reference operator [] (size_t id) const
    {
        return x[id];
    }

    reference operator [] (size_t id)
    {
        return x[id];
    }

    value_type at(size_t id)
    {
        return x[id];
    }

    value_type at(size_t id) const
    {
        return x[id];
    }

    const_pointer data () const
    {
        return x.data();
    }

    Point<CoordType, Dimension> operator + (const Point<CoordType, Dimension>& rhs)
    {
        Point<CoordType, Dimension> tmp;
        for (size_t i = 0; i < Dimension; ++i)
        {
            tmp[i] = x[i] + rhs[i];
        }
        return tmp;
    }

    Point<CoordType, Dimension> operator - (const Point<CoordType, Dimension>& rhs)
    {
        Point<CoordType, Dimension> tmp;
        for (size_t i = 0; i < Dimension; ++i)
        {
            tmp[i] = x[i] - rhs[i];
        }
        return tmp;
    }

    Point<CoordType, Dimension>& operator += (const Point<CoordType, Dimension>& rhs)
    {
        for (size_t i = 0; i < Dimension; ++i)
        {
            x[i] += rhs[i];
        }
        return *this;
    }

    Point<CoordType, Dimension>& operator / (size_t rhs)
    {
        for (size_t i = 0; i < Dimension; ++i)
        {
            x[i] /= rhs;
        }
        return *this;
    }

    Point() = default;
    Point(const Point& ) = default;
    Point& operator = (const Point& ) = default;
    Point(Point&& ) = default;
    Point& operator = (Point&& ) = default;
private:
    std::array<CoordType, Dimension> x;
};

template <typename CoordType>
using Point3D = Point<CoordType, 3>;

template <typename CoordType>
using Point2D = Point<CoordType, 2>;


} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_POINT_H
