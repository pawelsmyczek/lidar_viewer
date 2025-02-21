#ifndef LIDAR_VIEWER_IOSTREAM_H
#define LIDAR_VIEWER_IOSTREAM_H

#include "IoStreamBase.h"

#include <chrono>
#include <memory>

namespace lidar_viewer::dev
{

/// base for any I/O system
struct IoStream
{
//    ///brief templated constructor
//    template <typename IoType, typename ... Args>
//    IoStream(Args ... args);

    ///brief templated constructor
    template <typename IoType>
    IoStream(std::unique_ptr<IoType>&& ioStream);

    IoStream() = default;
    ~IoStream() noexcept;

    /// @brief tries to open i/o stream
    /// @param args arguments to a new i/o stream
    template<typename IoType, typename ... Args>
    void createAndOpen(Args ... args);

    /// @brief tries to open i/o stream
    void open();

    /// @brief tries to read data from an i/o stream up until a certain timeout expires
    /// @param ptr pointer to data to read from
    /// @param size size of data to read
    /// @param millis timeout
    unsigned int read(void* ptr, unsigned int size, const std::chrono::milliseconds millis) const noexcept(false);

    /// @brief write data to an i/o stream when available
    /// @param ptr pointer to data to write
    /// @param size size of data to write
    void write(const void* ptr, unsigned int size, bool discardOutput = false) const;

    /// @brief closes stream
    void close() const;

    IoStream(const IoStream& ) = delete;
    IoStream& operator = (const IoStream& ) = delete;
    IoStream(IoStream&& ) = delete;
    IoStream& operator = (IoStream&& ) = delete;

private:
    std::unique_ptr<IoStreamBase> ioStreamBase;
};

template <typename IoType>
IoStream::IoStream(std::unique_ptr<IoType>&& ioStream)
: ioStreamBase{std::move(ioStream)}
{
}

template<typename IoType, typename ... Args>
void IoStream::createAndOpen(Args ... args)
{
    std::unique_ptr<IoStreamBase> ioStream = std::make_unique<IoType>(args...);
    ioStreamBase.swap(ioStream);
    ioStreamBase->open();
}
} // namespace dev::lidar_viewer

#endif //LIDAR_VIEWER_IOSTREAM_H
