#ifndef LIDAR_VIEWER_DYNAMICMATRIX_H
#define LIDAR_VIEWER_DYNAMICMATRIX_H

#include "MatrixBase.h"

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lidar_viewer::geometry::types
{

/// Zero initialised matrix with dimensions chosen at run time.
/// @note a moved-from matrix keeps its dimensions but has no elements: only assign to it or
///       destroy it
/// @tparam T type of a single element, e.g. double
template <typename T>
struct DynamicMatrix
        : public MatrixBase<DynamicMatrix<T>, T>
{
    /// creates a matrix with all elements equal to zero
    /// @param rows number of rows
    /// @param cols number of columns
    DynamicMatrix(size_t rows, size_t cols)
    : arr(rows * cols)
    , m{rows}
    , n{cols}
    {}

    /// number of rows
    [[nodiscard]] size_t rows() const
    {
        return m;
    }

    /// number of columns
    [[nodiscard]] size_t cols() const
    {
        return n;
    }

    /// pointer to the first element, the elements are stored contiguously, row by row
    T* data()
    {
        return arr.data();
    }

    /// @copydoc data()
    const T* data() const
    {
        return arr.data();
    }

    /// dimensions of the matrix: (rows, columns)
    [[nodiscard]] std::pair<size_t, size_t> resolution() const
    {
        return std::make_pair(m, n);
    }

    /// Matrix product: (M x N) * (N x P) gives (M x P).
    /// @param rhs right operand, its number of rows must equal the number of columns of this matrix
    /// @return new matrix, the operands are not modified
    /// @throws std::runtime_error when the number of columns differs from the rhs number of rows
    DynamicMatrix operator * (const DynamicMatrix& rhs) const
    {
        if (n != rhs.m)
        {
            throw std::runtime_error{"Matrix dimensions do not match"};
        }
        DynamicMatrix tmp{m, rhs.n};
        detail::multiplyAccumulate(*this, rhs, tmp);
        return tmp;
    }

    /// Transpose: rows become columns, (M x N) gives (N x M). The matrix is not modified.
    /// @return new matrix with `result(y, x) == (*this)(x, y)` and the dimensions swapped
    DynamicMatrix transpose() const
    {
        DynamicMatrix out{cols(), rows()};
        detail::transposeInto(*this, out);
        return out;
    }

    /// Identity matrix: ones on the diagonal, zeros elsewhere.
    /// @param n number of rows and columns
    static DynamicMatrix identity(const size_t n)
    {
        DynamicMatrix tmp{n, n};
        detail::identity(tmp);
        return tmp;
    }

private:
    std::vector<T> arr;
    size_t m, n;
};

} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_DYNAMICMATRIX_H
