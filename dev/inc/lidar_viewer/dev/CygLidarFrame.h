#ifndef LIDAR_VIEWER_CYGLIDARFRAME_H
#define LIDAR_VIEWER_CYGLIDARFRAME_H

#include "IoStream.h"
#include "StatusOr.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <type_traits>
#include <stdexcept>

namespace lidar_viewer::dev
{

/// frame represents messages sent to/received from lidar,
/// s template parameter is size of payload together with payload header
/// by default frames are of BAD Status
template < uint16_t s >
class Frame
{
public:
    using Payload = std::array<uint8_t, s>;
    using Header = std::array<uint8_t, 3>;

    explicit constexpr Frame(Payload p_)
      : data{.repr{.payload_{p_}, .checksum_{checksum()}}},
        status_{Status::OK} {}

    explicit Frame(Status stat) : data{}, status_{stat} {}

    constexpr Frame() noexcept : data{}, status_{Status::OK}
    {}

    /// @returns pointer to the first element of the payload
    const Payload* payload() const noexcept
    { return &data.repr.payload_; }

    /// @returns header
    [[nodiscard]] Header header() const noexcept
    { return data.repr.header_; }

    /// @returns raw data size
    [[nodiscard]] auto rawSize() const noexcept
    { return data.raw.size(); }

    /// @returns frame status
    [[nodiscard]] Status status() const noexcept
    { return status_; }

    /// @returns pointer to the first element of a raw frame data
    [[nodiscard]] const uint8_t* raw() const noexcept
    {
        return &data.raw[0];
    }

    /// @returns size of frame's payload
    [[nodiscard]] inline auto size() const noexcept
    { return s; }

    /// @parameter inchecksum input checksum
    /// @returns false on validation failure, true otherwise
    [[nodiscard]] bool validateChecksum(const uint8_t inchecksum) const
    {
        return data.repr.checksum_ == inchecksum;
    }

    /// @brief calculate checksum on length and actual payload
    /// @returns checksum
    [[nodiscard]] uint8_t checksum() const noexcept
    {
        auto checkSum = 0u;
        const auto sizeOfHeader = data.repr.header_.size();
        std::for_each(data.raw.cbegin() + sizeOfHeader, data.raw.cend() - (sizeof(data.repr.checksum_)+1u),
                      [&checkSum](const auto& rawElement) { checkSum ^= rawElement; });
        return checkSum;
    }

  Frame(const Frame &) = default;
  Frame &operator=(const Frame &) = default;
  Frame(Frame &&) noexcept = default;
  Frame &operator=(Frame &&) noexcept = default;
  ~Frame() = default;

private:
    union
    {
        struct Repr
        {
            const Header header_ {0x5au, 0x77u, 0xffu};
            decltype(s) length_{s};
            Payload payload_{};
            uint8_t checksum_{};
        } __attribute__((packed)) repr{};
        std::array<uint8_t, sizeof(Repr)> raw;
    } data;
    Status status_;
};

/// @brief read frame from a serial port
/// @parameter serial reference to serial port
template < class RespFrame > static inline
StatusOr<RespFrame> read(IoStream& ioStream)
{
    using namespace std::chrono_literals;
    RespFrame respFrame;
    const auto expectedFrameHeader = respFrame.header();
    auto rawResponse = const_cast<uint8_t*>(respFrame.raw());
    typename std::remove_const<decltype(expectedFrameHeader)>::type retHeader{};

    try
    {
        if (ioStream.read(&retHeader[0], expectedFrameHeader.size(), 130ms);
                retHeader != expectedFrameHeader)
        {
            return Status::BAD;
        }

        auto readBytesInc = respFrame.header().size();
        rawResponse += readBytesInc;
        decltype(respFrame.size()) returnedSize{};

        if(readBytesInc = ioStream.read(reinterpret_cast<uint8_t *>(&returnedSize),
                                        sizeof(returnedSize), 1ms);
                readBytesInc != sizeof(returnedSize) || returnedSize != respFrame.size())
        {
            std::cerr << __func__ << ": " << readBytesInc
                        << " != " << sizeof(returnedSize) << " || "
                            << returnedSize <<" != " << respFrame.size() <<"\n";
            return Status::BAD;
        }

        rawResponse += readBytesInc;
        ioStream.read(rawResponse, returnedSize + 1, 6ms);
        return respFrame;
    }
    catch (const std::exception& e)
    {
        using namespace std::string_literals;
//        std::cerr << "Exception in frame read : " << e.what() << '\n';
        throw std::runtime_error{"Exception in frame read : "s + e.what()};
    }
}

/// @brief read frame from a serial port
/// @parameter serial reference to serial port
/// @parameter respFrame reference to output frame
template < class RespFrame > static inline
Status read(IoStream& ioStream, RespFrame& respFrame)
{
    using namespace std::chrono_literals;
    using ExpectedHeader = typename std::remove_const<decltype(RespFrame{}.header())>::type;
    auto rawResponse = const_cast<uint8_t*>(respFrame.raw());

    const auto expectedFrameHeader = respFrame.header();
    ExpectedHeader retHeader{};

    try
    {
        if (ioStream.read(&retHeader[0], expectedFrameHeader.size(), 130ms);
                retHeader != expectedFrameHeader)
        {
            return Status::BAD;
        }

        auto readBytesInc = respFrame.header().size();
        rawResponse += readBytesInc;
        decltype(respFrame.size()) returnedSize{};

        if(readBytesInc = ioStream.read(reinterpret_cast<uint8_t *>(&returnedSize),
                                        sizeof(returnedSize), 1ms);
                readBytesInc != sizeof(returnedSize))
        {
            std::cerr << __func__ << ": " << readBytesInc
                      << " != " << sizeof(returnedSize) <<"\n";
            return Status::BAD;
        }
        if(returnedSize != respFrame.size())
        {
            std::cerr << __func__ << ": " << returnedSize <<" != " << respFrame.size() <<"\n";
            returnedSize = respFrame.size();
            // TODO bad sized frames allowed since it breaks the binary file reader,
            // return StatusOr::Status::BAD;
        }

        rawResponse += readBytesInc;
        ioStream.read(rawResponse, returnedSize + 1, 6ms);
    }
    catch (const std::exception& e)
    {
        using namespace std::string_literals;
//        std::cerr << "Exception in frame read : " << e.what() << '\n';
        throw std::runtime_error{"Exception in frame read : \n"s + e.what()};
    }
    return Status::OK;
}

/// @brief read frame from a serial port
/// @parameter serial reference to serial port
/// @parameter respFrame pointer to output frame
template < class RespFrame > static inline
void read(IoStream& ioStream, RespFrame* respFrame)
{
    if ( !respFrame )
    {
        return ;
    }
    using namespace std::chrono_literals;
    const auto expectedFrameHeader = respFrame->header();
    auto rawResponse = const_cast<uint8_t*>(respFrame->raw());
    typename std::remove_const<decltype(expectedFrameHeader)>::type retHeader{};

    if (ioStream.read(&retHeader[0], expectedFrameHeader.size(), 130ms);
            retHeader != expectedFrameHeader)
    {
        return ;
    }

    auto readBytesInc = respFrame->header().size();
    rawResponse += readBytesInc;
    decltype(RespFrame::size()) returnedSize{};
    if(readBytesInc = ioStream.read(reinterpret_cast<uint8_t *>(&returnedSize),
                                    sizeof(returnedSize), 1ms);
            readBytesInc != sizeof(returnedSize) || returnedSize != respFrame->size())
    {
        std::cerr << __func__ << ": " << readBytesInc
                    << " != " << sizeof(returnedSize) << " || "
                        << returnedSize <<" != " << RespFrame::size() <<"\n";
        return ;
    }

    rawResponse += readBytesInc;
    ioStream.read(rawResponse, returnedSize - readBytesInc, 6ms);
}

/// @brief write frame to a serial port
/// @parameter writeFrame frame to write
/// @parameter serial reference to serial port
template < uint16_t s > static inline
void write(Frame<s>&& writeFrame, IoStream& ioStream)
{
    ioStream.write(writeFrame.raw(), writeFrame.rawSize());
}

/// @brief write frame to a serial port
/// @parameter writeFrame frame to write
/// @parameter serial reference to serial port
template < uint16_t s > static inline
void write(const Frame<s>& writeFrame, IoStream& ioStream)
{
    ioStream.write(writeFrame.raw(), writeFrame.rawSize());
}

} // namespace lidar_viewer::dev

#endif // LIDAR_VIEWER_CYGLIDARFRAME_H
