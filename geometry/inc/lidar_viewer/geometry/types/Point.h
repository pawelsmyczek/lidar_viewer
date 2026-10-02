#ifndef LIDAR_VIEWER_POINT_H
#define LIDAR_VIEWER_POINT_H

#include <array>
#include <utility>

namespace lidar_viewer::geometry::types
{

/// Fixed-size point with `Dimension` coordinates of type `CoordType`.
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
    constexpr explicit Point(const std::array<CoordType, Dimension>& il)
    : x{il}
    {}

    /// default constructed point has initialised coordinates
    constexpr Point()
        : x{}
    {
    }

    /// converts a point with another coordinate type, every coordinate is cast with static_cast
    /// (a same-type copy still uses the copy constructor)
    template <typename U>
    constexpr explicit Point(const Point<U, Dimension>& other)
        : Point{other, std::make_index_sequence<Dimension>{}}
    {}

    /// unchecked coordinate access, `id` must be lower than Dim
    constexpr const_reference operator [] (size_t id) const
    {
        return x[id];
    }

    constexpr reference operator [] (size_t id)
    {
        return x[id];
    }

    /// coordinate access by value (unchecked, like operator[])
    constexpr value_type at(size_t id)
    {
        return x[id];
    }

    constexpr value_type at(size_t id) const
    {
        return x[id];
    }

    /// pointer to the first coordinate, coordinates are stored contiguously
    constexpr const_pointer data () const
    {
        return x.data();
    }

    /// coordinate-wise sum, returns a new point
    constexpr Point<CoordType, Dimension> operator + (const Point<CoordType, Dimension>& rhs) const
    {
        Point<CoordType, Dimension> tmp{*this};
        tmp += rhs;
        return tmp;
    }

    /// coordinate-wise difference, returns a new point
    constexpr Point<CoordType, Dimension> operator - (const Point<CoordType, Dimension>& rhs) const
    {
        Point<CoordType, Dimension> tmp{*this};
        tmp -= rhs;
        return tmp;
    }

    /// adds `rhs` coordinate-wise to this point in place
    constexpr Point<CoordType, Dimension>& operator += (const Point<CoordType, Dimension>& rhs)
    {
        [this, &rhs]<size_t... Is>(std::index_sequence<Is...>)
        {
            ((x[Is] += rhs[Is]), ...);
        }(std::make_index_sequence<Dimension>{});
        return *this;
    }

    /// subtracts `rhs` coordinate-wise from this point in place
    constexpr Point<CoordType, Dimension>& operator -= (const Point<CoordType, Dimension>& rhs)
    {
        [this, &rhs]<size_t... Is>(std::index_sequence<Is...>)
        {
            ((x[Is] -= rhs[Is]), ...);
        }(std::make_index_sequence<Dimension>{});
        return *this;
    }

    /// divides every coordinate by `rhs` **in place** and returns a reference to this point,
    /// so, despite the operator, the receiver is modified (`rhs` must not be zero)
    constexpr Point<CoordType, Dimension>& operator / (size_t rhs)
    {
        [this, rhs]<size_t... Is>(std::index_sequence<Is...>)
        {
            ((x[Is] /= rhs), ...);
        }(std::make_index_sequence<Dimension>{});
        return *this;
    }

    Point(const Point& ) = default;
    Point& operator = (const Point& ) = default;
    Point(Point&& ) = default;
    Point& operator = (Point&& ) = default;
private:
    /// expands the indices, so that all the coordinates are cast in one braced initialisation
    template <typename U, size_t... Is>
    constexpr Point(const Point<U, Dimension>& other, std::index_sequence<Is...>)
        : x{static_cast<CoordType>(other[Is])...}
    {}

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
