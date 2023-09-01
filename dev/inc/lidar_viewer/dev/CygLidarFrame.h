#ifndef LIDAR_VIEWER_CYGLIDARFRAME_H
#define LIDAR_VIEWER_CYGLIDARFRAME_H

#include "SerialPort.h"
#include "StatusOr.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>

namespace lidar_viewer::dev
{

/// frame represents messages sent to/received from lidar,
/// s template parameter is size of payload together with payload header
template < uint16_t s >
class Frame
{
public:
    enum class Status
    {
        OK = 0u,
        BAD
    };
    using Payload = std::array<uint8_t, s>;
    using Header = std::array<uint8_t, 3>;

  explicit constexpr Frame(Payload p_)
      : data{.repr{.payload_{p_}, .checksum_{checksum()}}},
        status_{StatusOr::Status::OK} {}

  explicit Frame(StatusOr::Status stat) : data{}, status_{stat} {}

  Frame() : data{}, status_{StatusOr::Status::OK} {}

    /// @returns pointer to the first element of the payload
    const Payload* payload() const noexcept
    { return &data.repr.payload_; }

    /// @returns header
    [[nodiscard]] Header header() const noexcept
    { return data.repr.header_; }

    /// @returns payload length
    [[nodiscard]] uint16_t length() const noexcept
    { return data.repr.length_; }

    /// @returns frame status
    [[nodiscard]] Status status() const noexcept
    { return status_; }

    [[nodiscard]] const uint8_t* raw() const noexcept
    {
        return &data.raw[0];
    }

    /// performs integrity check via checksum calculation
    [[nodiscard]] bool validateChecksum(const uint8_t inchecksum) const
    {
        return data.repr.checksum_ == inchecksum;
    }

    [[nodiscard]] uint8_t checksum() const
    {
        auto checkSum = 0u;
        for ( auto i = 3u; i <= data.raw.size()-3u; ++i )
        {
            checkSum ^= data.raw[i];
        }
        return checkSum;
    }

  Frame(const Frame &) = default;
  Frame &operator=(const Frame &) = default;
  Frame(Frame &&) noexcept = default;
  Frame &operator=(Frame &&) noexcept = default;
  ~Frame() = default;

private:
    union Data
    {
        struct Repr
        {
            const Header header_ {0x5au, 0x77u, 0xffu};
            decltype(s) length_{s};
            Payload payload_;
            uint8_t checksum_{};
        } __attribute__((packed)) repr{};
        std::array<uint8_t, sizeof(Repr)> raw;
    } data;
    StatusOr::Status status_;
};

/// @brief read frame from a serial port
/// @parameter serial reference to serial port
template < class RespFrame >
RespFrame read(SerialPort& serial)
{
    using namespace std::chrono_literals;
    RespFrame respFrame;
    const auto expectedFrameHeader = respFrame.header();
    auto rawResponse = const_cast<uint8_t*>(respFrame.raw());
    typename std::remove_const<decltype(expectedFrameHeader)>::type retHeader{};

    if (serial.read(&retHeader[0], expectedFrameHeader.size(), 60ms);
            retHeader != expectedFrameHeader)
    {
        return StatusOr::MakeStatusOrError<RespFrame>(
                StatusOr::Status::BAD);
    }

    auto readBytesInc = respFrame.header().size();
    rawResponse += readBytesInc;
    decltype(RespFrame::size()) returnedSize{};

    if(readBytesInc = serial.read(reinterpret_cast<uint8_t *>(&returnedSize),
                                                        sizeof(returnedSize), 1ms);
            readBytesInc != sizeof(returnedSize) || returnedSize != RespFrame::size())
    {
        std::cerr << __func__ << ": " << readBytesInc
                    << " != " << sizeof(returnedSize) << " || "
                        << returnedSize <<" >= " << RespFrame::size() <<"\n";
        return StatusOr::MakeStatusOrError<RespFrame>(
                StatusOr::Status::BAD);
    }

    rawResponse += readBytesInc;
    serial.read(rawResponse, returnedSize - readBytesInc, 6ms);
    return respFrame;
}


/// @brief write frame to a serial port
/// @parameter writeFrame frame to write
/// @parameter serial reference to serial port
template < uint16_t s >
void write(Frame<s> writeFrame, SerialPort& serial)
{
    serial.write(writeFrame.raw(), sizeof(Frame<s>)-sizeof(typename Frame<s>::Status));
}

} // namespace lidar_viewer::dev

#endif // LIDAR_VIEWER_CYGLIDARFRAME_H
