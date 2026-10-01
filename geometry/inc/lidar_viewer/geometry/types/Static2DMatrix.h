#ifndef LIDAR_VIEWER_STATIC2DMATRIX_H
#define LIDAR_VIEWER_STATIC2DMATRIX_H

#include "MatrixBase.h"

#include <array>
#include <cstddef>

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

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_STATIC2DMATRIX_H
