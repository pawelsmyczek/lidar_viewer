#ifndef LIDAR_VIEWER_CYGLIDARFRAME_H
#define LIDAR_VIEWER_CYGLIDARFRAME_H

#include "SerialPort.h"

#include <chrono>
#include <iostream>
#include <cstdint>
#include <array>

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
            : data{
            .repr{ .payload_{p_}, .checksum_{checksum()}
            }
    }
            , status_{Status::OK}
    {}

    explicit Frame(Status stat) : data{}, status_{stat}
    {}

    Frame() : data{}, status_{Status::OK}
    {}

    /// @brief write frame to a serial port
    void write(SerialPort& sp);

    /// @brief read frame from a serial port
    auto read(SerialPort& serial);

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
        for ( auto i = 3; i <= data.raw.size()-3; ++i )
        {
            checkSum ^= data.raw[i];
        }
        return checkSum;
    }

    Frame(const Frame&) = default;
    Frame& operator = (const Frame&) = default;
    Frame(Frame&&)  noexcept = default;
    Frame& operator = (Frame&&)  noexcept = default;
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
    Status status_;
};

template < uint16_t s >
auto Frame<s>::read(SerialPort& serial)
{
    using namespace std::chrono_literals;
    using RespFrame = Frame<s>;

    const auto expectedFrameHeader = header();
    auto rawResponse = const_cast<uint8_t*>(raw());
    typename std::remove_const<decltype(expectedFrameHeader)>::type retHeader{};

    if (serial.read(&retHeader[0], expectedFrameHeader.size(), 2ms);
            retHeader != expectedFrameHeader)
    {
        return RespFrame {Status::BAD};
    }

    auto readBytesInc = header().size();
    rawResponse += readBytesInc;
    decltype(s) returnedSize{};

    if(readBytesInc = serial.read(reinterpret_cast<uint8_t *>(&returnedSize), sizeof(returnedSize), 1ms);
            readBytesInc != sizeof(returnedSize) || returnedSize != s)
    {
        std::cerr << __func__ << ": " << readBytesInc << " != " << sizeof(returnedSize) << " || "<< returnedSize <<" >= " << s <<"\n";
        return RespFrame {Status::BAD};
    }

    rawResponse += readBytesInc;
    serial.read(rawResponse, returnedSize - readBytesInc, 6ms);
    return *this;
}

template < uint16_t s >
void Frame<s>::write(SerialPort& sp)
{
    using ReqFrame = Frame<s>;
    sp.write(raw(), sizeof(ReqFrame)-sizeof(typename ReqFrame::Status));
}


} // lidar_viewer::dev

#endif //LIDAR_VIEWER_CYGLIDARFRAME_H
