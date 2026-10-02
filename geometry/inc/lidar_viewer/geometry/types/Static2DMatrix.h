#ifndef LIDAR_VIEWER_STATIC2DMATRIX_H
#define LIDAR_VIEWER_STATIC2DMATRIX_H

#include "MatrixBase.h"
#include "Vector.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <numeric>

namespace lidar_viewer::geometry::types
{

/// Zero initialised matrix with M rows and N columns known at compile time.
/// @tparam T type of a single element, e.g. double
/// @tparam M number of rows
/// @tparam N number of columns
template <typename T, size_t M, size_t N>
struct Static2DMatrix
        : public MatrixBase<Static2DMatrix<T, M, N>, T>
{
    /// creates a matrix with all elements equal to zero
    Static2DMatrix() = default;

    /// creates a matrix from `M * N` values given row by row
    /// @param arr the elements, `arr[row * N + column]`
    explicit Static2DMatrix(const std::array<T, M * N>& arr)
        : arr{arr}
    { }

    /// Creates a matrix whose rows are the given vectors: `fromRows(r0, r1, r2)` has `r0` as its
    /// first row. The number of vectors must be `M` and each must have `N` coordinates of type
    /// `T`, which is checked at compile time.
    template <VectorOf<T, N>... Vectors>
        requires (sizeof...(Vectors) == M)
    static Static2DMatrix fromRows(const Vectors&... rows)
    {
        Static2DMatrix result;
        size_t y = 0;
        (std::copy_n(rows.data(), N, result.row(y++).begin()), ...);
        return result;
    }

    /// Creates a matrix whose columns are the given vectors: `fromColumns(c0, c1, c2)` has `c0` as
    /// its first column. The number of vectors must be `N` and each must have `M` coordinates of
    /// type `T`, which is checked at compile time. A matrix maps the `i`-th unit vector to its
    /// `i`-th column.
    template <VectorOf<T, M>... Vectors>
        requires (sizeof...(Vectors) == N)
    static Static2DMatrix fromColumns(const Vectors&... columns)
    {
        Static2DMatrix result;
        size_t x = 0;
        const auto setColumn = [&result, &x](const Vector<T, M>& column)
        {
            for (const auto y : std::views::iota(size_t{0}, M))
            {
                result(x, y) = column[y];
            }
            ++x;
        };
        (setColumn(columns), ...);
        return result;
    }

    /// number of rows, `M`
    static constexpr size_t rows()
    {
        return M;
    }

    /// number of columns, `N`
    static constexpr size_t cols()
    {
        return N;
    }

    /// pointer to the first element, the `M * N` elements are stored contiguously, row by row
    T* data()
    {
        return arr.data();
    }

    /// @copydoc data()
    const T* data() const
    {
        return arr.data();
    }

    /// Matrix product: (M x N) * (N x P) gives (M x P). The inner dimensions are checked at
    /// compile time.
    /// @tparam P number of columns of `rhs` and of the result
    /// @param rhs right operand
    /// @return new matrix, the operands are not modified
    template <size_t P>
    Static2DMatrix<T, M, P> operator * (const Static2DMatrix<T, N, P>& rhs) const
    {
        Static2DMatrix<T, M, P> tmp;
        detail::multiplyAccumulate(*this, rhs, tmp);
        return tmp;
    }

    /// Matrix times vector: (M x N) * N coordinates gives M coordinates, element `y` of the result
    /// is the dot product of row `y` with `v`. The size of the vector is checked at compile time.
    /// @param v the vector, it is not modified
    /// @return new vector
    Vector<T, M> operator * (const Vector<T, N>& v) const
    {
        Vector<T, M> result;
        for (const auto y : std::views::iota(size_t{0}, M))
        {
            const auto row = this->row(y);
            result[y] = std::inner_product(row.begin(), row.end(), v.data(), T{0});
        }
        return result;
    }

    /// Transpose: rows become columns, (M x N) gives (N x M). The matrix is not modified.
    /// @return new matrix with `result(y, x) == (*this)(x, y)`
    Static2DMatrix<T, N, M> transpose() const
    {
        Static2DMatrix<T, N, M> out;
        detail::transposeInto(*this, out);
        return out;
    }

    /// Identity matrix: ones on the diagonal, zeros elsewhere. Only available for square matrices,
    /// a non-square one fails to compile.
    static Static2DMatrix<T, N, M> identity()
    {
        Static2DMatrix<T, N, M> tmp{};
        static_assert(N == M, "identity can be performed only on a square matrix");
        detail::identity(tmp);
        return tmp;
    }

private:
    std::array<T, M * N> arr{};
};

/// Outer product of two vectors, `a * bᵀ`: an (M x N) matrix with `result(x, y) == a[y] * b[x]`,
/// i.e. row `y` is `b` scaled by `a[y]`. The cross-covariance of two point sets is a sum of these.
/// @param a the left vector, M coordinates (it becomes the column space of the result)
/// @param b the right vector, N coordinates
template <typename T, size_t M, size_t N>
Static2DMatrix<T, M, N> outer(const Vector<T, M>& a, const Vector<T, N>& b)
{
    Static2DMatrix<T, M, N> result;
    for (const auto y : std::views::iota(size_t{0}, M))
    {
        auto row = result.row(y);
        std::transform(b.data(), b.data() + N, row.begin(),
                       [factor = a[y]](const T value) { return factor * value; });
    }
    return result;
}

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_STATIC2DMATRIX_H
