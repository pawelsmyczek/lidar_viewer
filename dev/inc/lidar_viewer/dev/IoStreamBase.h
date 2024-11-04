#ifndef LIDAR_VIEWER_IOSTREAMBASE_H
#define LIDAR_VIEWER_IOSTREAMBASE_H

#include <chrono>

namespace lidar_viewer::dev
{

struct IoStreamBase
{
    IoStreamBase() = default;
    virtual ~IoStreamBase() noexcept = default;

    /// @brief tries to  an i/o stream
    virtual void open() noexcept(false) = 0;

    /// @brief tries to read data from an i/o stream up until a certain timeout expires
    /// @param ptr pointer to data to read from
    /// @param size size of data to read
    /// @param millis timeout
    virtual unsigned int read(void* ptr, unsigned int size, const std::chrono::milliseconds millis) const noexcept(false) = 0;

    /// @brief write data to an i/o stream when available
    /// @param ptr pointer to data to write
    /// @param size size of data to write
    virtual void write(const void* ptr, unsigned int size, bool discardOutput = false) const = 0;

    /// @brief closes stream
    virtual void close() const = 0;

    IoStreamBase(const IoStreamBase& ) = delete;
    IoStreamBase& operator = (const IoStreamBase& ) = delete;
    IoStreamBase(IoStreamBase&& ) = delete;
    IoStreamBase& operator = (IoStreamBase&& ) = delete;

};

} // namespace lidar_viewer::dev

#endif //LIDAR_VIEWER_IOSTREAMBASE_H
