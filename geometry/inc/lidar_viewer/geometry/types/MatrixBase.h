#ifndef LIDAR_VIEWER_MATRIXBASE_H
#define LIDAR_VIEWER_MATRIXBASE_H

#include <algorithm>
#include <cstddef>
#include <functional>
#include <ranges>
#include <span>
#include <stdexcept>

namespace lidar_viewer::geometry::types
{

// schoolbook matrix implementation, there is room for improvements

/// CRTP base of the matrices: bounds-checked access and element-wise operations.
/// `Derived` provides `rows()`, `cols()` and `data()` (elements stored row by row).
/// Elements are addressed as `(x, y)`: `x` is the column, `y` the row.
/// @tparam Derived the matrix class deriving from this base
/// @tparam T type of a single element, e.g. double
template <typename Derived, typename T>
struct MatrixBase
{
    using value_type = T;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;

    /// Bounds-checked access to an element by its flat index.
    /// @param id element number, counted row by row, must be lower than size()
    /// @return reference to the element
    /// @throws std::runtime_error when `id` is past the end
    reference get(size_t id)
    {
        checkIndex(id);
        return self().data()[id];
    }

    /// @copydoc get(size_t)
    const_reference get(size_t id) const
    {
        checkIndex(id);
        return self().data()[id];
    }

    /// Bounds-checked element access. Both coordinates are checked separately, so a column past the
    /// end is an error and does not wrap around to the next row.
    /// @param x column, must be lower than cols()
    /// @param y row, must be lower than rows()
    /// @return reference to the element
    /// @throws std::runtime_error when `x` or `y` is outside of the matrix
    reference operator()(size_t x, size_t y)
    {
        return self().data()[flatIndex(x, y)];
    }

    /// @copydoc operator()(size_t,size_t)
    const_reference operator()(size_t x, size_t y) const
    {
        return self().data()[flatIndex(x, y)];
    }

    /// Start of the range of all elements, in row-major order (row after row), so the matrix can be
    /// used with range-based for loops and standard algorithms.
    pointer begin()
    {
        return self().data();
    }

    /// One past the last element, see begin()
    pointer end()
    {
        return self().data() + size();
    }

    /// @copydoc begin()
    const_pointer begin() const
    {
        return self().data();
    }

    /// @copydoc end()
    const_pointer end() const
    {
        return self().data() + size();
    }

    /// Bounds-checked view of one row. The span refers to the matrix elements, it is not a copy, and
    /// it must not outlive the matrix.
    /// @param y row, must be lower than rows()
    /// @return span of cols() elements
    /// @throws std::runtime_error when `y` is past the last row
    std::span<T> row(size_t y)
    {
        checkRow(y);
        return {self().data() + y * self().cols(), self().cols()};
    }

    /// @copydoc row(size_t)
    std::span<const T> row(size_t y) const
    {
        checkRow(y);
        return {self().data() + y * self().cols(), self().cols()};
    }

    /// Element-wise sum, the operands are not modified.
    /// @param rhs matrix of the same dimensions
    /// @return new matrix with `(*this)(x, y) + rhs(x, y)` at every position
    /// @throws std::runtime_error when the dimensions differ (only possible for matrices with
    ///         run time dimensions, the dimensions of static matrices are part of their type)
    Derived operator + (const Derived& rhs) const
    {
        checkSameShape(rhs);
        Derived tmp{self()};
        std::transform(tmp.begin(), tmp.end(), rhs.begin(), tmp.begin(), std::plus<>{});
        return tmp;
    }

    /// Element-wise difference, the operands are not modified.
    /// @param rhs matrix of the same dimensions
    /// @return new matrix with `(*this)(x, y) - rhs(x, y)` at every position
    /// @throws std::runtime_error when the dimensions differ (only possible for matrices with
    ///         run time dimensions, the dimensions of static matrices are part of their type)
    Derived operator - (const Derived& rhs) const
    {
        checkSameShape(rhs);
        Derived tmp{self()};
        std::transform(tmp.begin(), tmp.end(), rhs.begin(), tmp.begin(), std::minus<>{});
        return tmp;
    }

    /// total number of elements, `rows() * cols()`
    [[nodiscard]] size_t size() const
    {
        return self().rows() * self().cols();
    }

    MatrixBase() = default;
    MatrixBase(const MatrixBase&) = default;
    MatrixBase& operator = (const MatrixBase&) = default;
    MatrixBase(MatrixBase&&) = default;
    MatrixBase& operator = (MatrixBase&&) = default;
    // not virtual and not public: the base is never deleted or used through a base pointer
    ~MatrixBase() = default;

private:
    Derived& self()
    {
        return static_cast<Derived&>(*this);
    }

    const Derived& self() const
    {
        return static_cast<const Derived&>(*this);
    }

    void checkIndex(size_t id) const
    {
        if (id >= size())
        {
            throw std::runtime_error{"Access to array past the end"};
        }
    }

    void checkRow(size_t y) const
    {
        if (y >= self().rows())
        {
            throw std::runtime_error{"Access to array past the end"};
        }
    }

    [[nodiscard]] size_t flatIndex(size_t x, size_t y) const
    {
        if (x >= self().cols() || y >= self().rows())
        {
            throw std::runtime_error{"Access to array past the end"};
        }
        return y * self().cols() + x;
    }

    void checkSameShape(const Derived& rhs) const
    {
        if (self().rows() != rhs.rows() || self().cols() != rhs.cols())
        {
            throw std::runtime_error{"Matrix dimensions do not match"};
        }
    }
};

namespace detail
{

/// `out += lhs * rhs`, computed row by row.
/// @note dimensions are not checked: `out` must be (lhs.rows() x rhs.cols()) and
///       `lhs.cols()` must equal `rhs.rows()`
template <typename Lhs, typename Rhs, typename Out>
void multiplyAccumulate(const Lhs& lhs, const Rhs& rhs, Out& out)
{
    for (const auto y : std::views::iota(size_t{0}, lhs.rows()))
    {
        auto outRow = out.row(y);
        for (const auto k : std::views::iota(size_t{0}, lhs.cols()))
        {
            const auto factor = lhs(k, y);
            const auto rhsRow = rhs.row(k);
            std::transform(rhsRow.begin(), rhsRow.end(), outRow.begin(), outRow.begin(),
                           [factor](const auto& r, const auto& o) { return o + factor * r; });
        }
    }
}

/// Matrix transposition shared by the matrix classes: writes the transpose of `in` to `out`,
/// i.e. `out(y, x) = in(x, y)` for every element. Complexity is O(in.rows() * in.cols()).
/// @note `out` must be (in.cols() x in.rows()), i.e. have the dimensions swapped, which is not
///       checked beyond the bounds checks of the element access, the callers make sure of it.
/// @tparam In matrix type of the source
/// @tparam Out matrix type of the result
template <typename In, typename Out>
void transposeInto(const In& in, Out& out)
{
    for (const auto y : std::views::iota(size_t{0}, in.rows()))
    {
        for (const auto k : std::views::iota(size_t{0}, in.cols()))
        {
            out(y, k) = in(k, y);
        }
    }
}

/// sets the diagonal of a zero initialised square matrix to one
/// @note squareness is not checked, the callers make sure of it
template <typename Out>
void identity(Out& out)
{
    for (const auto y : std::views::iota(size_t{0}, out.rows()))
    {
        auto tmpRow = out.row(y);
        tmpRow[y] = 1;
    }
}

} // namespace detail

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_MATRIXBASE_H
