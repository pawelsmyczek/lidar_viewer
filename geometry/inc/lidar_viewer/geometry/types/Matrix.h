#ifndef LIDAR_VIEWER_MATRIX_H
#define LIDAR_VIEWER_MATRIX_H

#include <array>
#include <stdexcept>
#include <vector>

namespace lidar_viewer::geometry::types
{

// schoolbook matrix implementation, there is room for improvements

template <typename T>
struct Matrix
{
    using value_type = T;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;

    explicit Matrix(pointer in, size_t N)
    : data{in}
    , size{N}
    { }

    reference get(size_t id) noexcept(false)
    {
        if(id >= size )
        {
            throw std::runtime_error{"Access to array past the end"};
        }
        return data[id];
    }

    virtual ~Matrix() = default;
    Matrix() = default;
    Matrix(Matrix&&) = default;
    Matrix& operator =(Matrix&&) = default;
    Matrix(const Matrix&) = default;
    Matrix& operator =(const Matrix&) = default;

protected:
    virtual void clone() = 0;

private:
    T* data;
    size_t size;
};



template <typename T, size_t M, size_t N>
struct Static2DMatrix
        : public Matrix<T>
{
    using value_type = T;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;

    Static2DMatrix()
    : Matrix<T>(arr.data(), M*N)
    , arr{}
    {}

    reference operator()(size_t x, size_t y)
    {
        return this->get(y * N + x);
    }

    const_reference operator()(size_t x, size_t y) const
    {
        return this->get(y * N + x);
    }

    template <size_t P>
    Static2DMatrix<T, M, P> operator * (const Static2DMatrix<T, N, P> & rhs) const
    {
        Static2DMatrix<T, M, P> tmp;
        for(size_t i = 0; i < M; ++i)
        {
            for(size_t j = 0; j < P; ++j)
            {
                for(size_t k = 0; k < N; ++k)
                {
                    tmp(i, j) += this->operator()(i, k) * rhs(k, j);
                }
            }
        }
        return tmp;
    }

    Static2DMatrix<T, M, N> operator + (const Static2DMatrix<T, M, N> & rhs) const
    {
        Static2DMatrix<T, M, N> tmp;
        for(size_t i = 0; i < M; ++i)
        {
            for(size_t j = 0; j < N; ++j)
            {
                tmp(i, j) = this->operator()(i, j) + rhs(i, j);
            }
        }
        return tmp;
    }

    Static2DMatrix<T, M, N> operator - (const Static2DMatrix<T, M, N> & rhs) const
    {
        Static2DMatrix<T, M, N> tmp;
        for(size_t i = 0; i < M; ++i)
        {
            for(size_t j = 0; j < N; ++j)
            {
                tmp(i, j) = this->operator()(i, j) - rhs(i, j);
            }
        }
        return tmp;
    }

    Static2DMatrix(const Static2DMatrix& rhs)
    {
        this = rhs.clone();
    }

    Static2DMatrix& operator = (const Static2DMatrix& rhs)
    {
        this = rhs.clone();
        return *this;
    }

    Static2DMatrix(Static2DMatrix&& ) = default;
    Static2DMatrix& operator = (Static2DMatrix&&) = default;
    ~Static2DMatrix() override = default;

private:

    Static2DMatrix<T, M, N>* clone()
    {
        return new Static2DMatrix<T, M, N>(*this);
    }

    std::array<T, M*N> arr;
};

template <typename T>
struct DynamicMatrix
        : public Matrix<T>
{
    using value_type = T;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;

    DynamicMatrix(const size_t m_, const size_t n_)
            : Matrix<T>(arr.data(), m_*n_)
            , arr{}
            , m{m_}
            , n{n_}
    { arr.reserve(n_*m_); }

    reference operator()(size_t x, size_t y)
    {
        return this->get(y * n + x);
    }

    const_reference operator()(size_t x, size_t y) const
    {
        return this->get(y * n + x);
    }

    DynamicMatrix operator * (const DynamicMatrix & rhs) const
    {
        const auto p  = rhs.resolution().second;
        DynamicMatrix tmp{m, p};//Static2DMatrix<T, M, P> tmp;
        for(size_t i = 0; i < m; ++i)
        {
            for(size_t j = 0; j < p; ++j)
            {
                for(size_t k = 0; k < n; ++k)
                {
                    tmp(i, j) += this->operator()(i, k) * rhs(k, j);
                }
            }
        }
        return tmp;
    }

    DynamicMatrix operator + (const DynamicMatrix & rhs) const
    {
        DynamicMatrix tmp{m,n};
        for(size_t i = 0; i < m; ++i)
        {
            for(size_t j = 0; j < n; ++j)
            {
                tmp(i, j) = this->operator()(i, j) + rhs(i, j);
            }
        }
        return tmp;
    }

    DynamicMatrix operator - (const DynamicMatrix & rhs) const
    {
        DynamicMatrix tmp{m,n};
        for(size_t i = 0; i < m; ++i)
        {
            for(size_t j = 0; j < n; ++j)
            {
                tmp(i, j) = this->operator()(i, j) - rhs(i, j);
            }
        }
        return tmp;
    }

    DynamicMatrix(const DynamicMatrix& rhs)
    {
        this = rhs.clone();
    }

    DynamicMatrix& operator = (const DynamicMatrix& rhs)
    {
        this = rhs.clone();
        return *this;
    }

    std::pair<size_t, size_t> resolution() const
    {
        std::make_pair(m, n);
    }

    DynamicMatrix(DynamicMatrix&& ) = default;
    DynamicMatrix& operator = (DynamicMatrix&&) = default;
    ~DynamicMatrix() override = default;

private:
    std::vector<T> arr;
    size_t m, n;
};



} // namespace lidar_viewer::geometry::types

#endif //LIDAR_VIEWER_MATRIX_H
