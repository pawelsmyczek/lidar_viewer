#ifndef LIDAR_VIEWER_DEV_STATUSOR_H
#define LIDAR_VIEWER_DEV_STATUSOR_H

#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace lidar_viewer::dev
{
enum class Status { OK = 0u, BAD };

/* Example usage
StatusOr<int> Divide(int a, int b) {
    if (b == 0) {
        return MakeStatusOrError<int>("Division by zero");
    }
    return MakeStatusOr<int>(a / b);
}

int main() {
    auto result = Divide(10, 2);
    if (result.ok()) {
        std::cout << "Result: " << result.value() << std::endl;
    } else {
        std::cout << "Error: " << result.error_message() << std::endl;
    }

    auto error_result = Divide(10, 0);
    if (error_result.ok()) {
        std::cout << "Result: " << error_result.value() << std::endl;
    } else {
        std::cout << "Error: " << error_result.error_message() << std::endl;
    }

    return 0;
}
*/
template <typename T>
class StatusOr
{
public:
    // Constructors for success or error
    StatusOr(const T &value) : has_value_(true), value_(value) {}
    StatusOr(T &&value) : has_value_(true), value_(std::move(value)) {}
    StatusOr(const Status &error) : has_value_(false), error_(error) {}

    // Accessors
    bool ok() const { return has_value_; }

    const T &value() const
    {
        if (!ok())
        {
            throw std::runtime_error(
                "Attempted to access value of an error StatusOr");
        }
        return value_;
    }

    T &value()
    {
        if (!ok())
        {
            throw std::runtime_error(
                "Attempted to access value of an error StatusOr");
        }
        return value_;
    }

    const Status &error() const
    {
        if (ok())
        {
            throw std::runtime_error(
            "Attempted to access error message of a valid StatusOr");
        }
        return error_;
    }

private:
    bool has_value_;
    T value_;
    Status error_;
};

// Helper functions to create StatusOr objects
template <typename T> StatusOr<T> MakeStatusOr(const T &value) {
  return StatusOr<T>(value);
}

template <typename T> StatusOr<T> MakeStatusOr(T &&value) {
  return StatusOr<T>(std::move(value));
}

template <typename T> StatusOr<T> MakeStatusOrError(const Status &error) {
  return StatusOr<T>(error);
}

} // namespace StatusOr

#endif // LIDAR_VIEWER_DEV_STATUSOR_H