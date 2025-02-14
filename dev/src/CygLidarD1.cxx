#include "lidar_viewer/dev/CygLidarD1.h"
#include "lidar_viewer/dev/CygLidarFrame.h"

#include <iostream>
#include <stdexcept>

using namespace std::chrono_literals;
using namespace std::string_literals;

namespace
{

enum RequestTypes
{
    DeviceInfo = 0x10u,
    Run2DMOde = 0x1u,
    Run3DMode = 0x8u,
    RunDualMode = 0x7u,
    Stop = 0x2u,
    SetLightPulseDuration = 0xCu,
    SetFreqChannel = 0xFu,
    SetSensitivity = 0x11u,
    BaudRate = 0x12u
};


}

namespace lidar_viewer::dev
{

CygLidarD1::PulseDuration::PulseDuration(
        PulseMode pulseMode, uint16_t duration_)
        : duration{duration_}
{
    duration |= static_cast<uint16_t>(pulseMode);
}

CygLidarD1::PulseDuration::PulseDuration()
: duration{static_cast<uint16_t>(PulseMode::AutoDual)}
{ }

uint16_t CygLidarD1::PulseDuration::get() const
{
    return duration;
}

CygLidarD1::CygLidarD1(IoStream& _ioStream)
: ioStream{_ioStream}
, frame3D{}
, frame2D{}
, pointcloud3d{}
, pointcloud2d{}
, failureInRead{false}
{ }

CygLidarD1::~CygLidarD1() noexcept
{
    stop();
}

std::ostream& operator << (std::ostream& outStream, const Frame<7>& deviceInfo)
{
    for ( const auto value: *deviceInfo.payload() )
    {
        outStream << std::to_string(value) << '.';
    }
    return outStream;
}

void CygLidarD1::printDeviceInfo()
{
    using Resp = Frame<7u>;
    write(Req2({RequestTypes::DeviceInfo, 0x00u}), ioStream);

    auto ret = read<Resp>(ioStream);
    if (ret.ok())
    {
        auto& retPayload = ret.value();
        std::cout << "Device info: " << retPayload << "\n";
    }
    return ;
}

void CygLidarD1::configure(const Config cfg)
{
    write( Req2({RequestTypes::BaudRate, static_cast<uint8_t>(cfg.baudRate)} ), ioStream );
    const auto pulseDuration = cfg.pulseDuration.get();
    write( Req3({RequestTypes::SetLightPulseDuration, static_cast<uint8_t>(pulseDuration & 0xffu),
            static_cast<uint8_t>((pulseDuration & 0xff00u) >> 8u)} ), ioStream );
    write( Req2({RequestTypes::SetFreqChannel, static_cast<uint8_t>(cfg.frequencyCh)} ), ioStream );
    write( Req2({RequestTypes::SetSensitivity, static_cast<uint8_t>(cfg.sensitivity)} ), ioStream );
}

void CygLidarD1::run(Mode mode)
{
    write(Req2( {static_cast<uint8_t>(mode), 0x00u} ), ioStream);
}

void CygLidarD1::readAndParse3dFrame()
{
    std::lock_guard lGuard{rwMutex};
    auto parsingFunction = [](auto& pointCloud, const auto& returnedPayload )
    {
        auto frame3dIt = pointCloud.begin();
        auto returnedFrameIt = (returnedPayload->begin())+1;
        for ( ; frame3dIt != pointCloud.end() && returnedFrameIt != returnedPayload->end();
                ++frame3dIt, ++returnedFrameIt )
        {
            const auto firstEl = *returnedFrameIt;
            const auto secondEl = *(++returnedFrameIt);
            const auto thirdEl = *(++returnedFrameIt);

            *frame3dIt = firstEl;
            *frame3dIt |= ((secondEl & 0xfu) << 8u);
            frame3dIt++;
            *frame3dIt = ((secondEl & 0xf0u) >> 4u);
            *frame3dIt |= (thirdEl << 4u);
        }
    };
    readAndParseFrame(frame3D, parsingFunction, pointcloud3d);
}

void CygLidarD1::readAndParse2dFrame()
{
    std::lock_guard lGuard{rwMutex};
    auto parsingFunction = [](auto& pointCloud, const auto& returnedPayload )
    {
        auto frame2dIt = pointCloud.begin();
        auto returnedFrameIt = (returnedPayload->begin())+1;
        for ( ; frame2dIt != pointCloud.end() && returnedFrameIt != returnedPayload->end();
                ++frame2dIt, ++returnedFrameIt )
        {
            const auto firstEl = *returnedFrameIt;
            const auto secondEl = *(++returnedFrameIt);

            *frame2dIt = firstEl;
            *frame2dIt |= ((secondEl & 0xfu) << 8u);
        }
    };
    readAndParseFrame( frame2D , parsingFunction, pointcloud2d);
}

void CygLidarD1::use2dPointCloud(CygLidarD1::PointCloud2DAccessorFunction&& accessor2d) const
{
    std::lock_guard lGuard{rwMutex};
    if(!accessor2d)
    {
        return ;
    }
    accessor2d(pointcloud2d);
}

void CygLidarD1::use3dPointCloud(CygLidarD1::PointCloud3DAccessorFunction&& accessor3d) const
{
    std::lock_guard lGuard{rwMutex};
    if (!accessor3d)
    {
        return;
    }
    accessor3d(pointcloud3d);
}

void CygLidarD1::stop()
{
    write(Req2( {0x02, 0x00u} ),ioStream);
}

void CygLidarD1::use3dFrame(CygLidarD1::Frame3DAccessorFunction &&accessor3d) const
{
    std::lock_guard lGuard{rwMutex};
    if (!accessor3d)
    {
        return;
    }
    accessor3d(frame3D);
}

void CygLidarD1::use2dFrame(CygLidarD1::Frame2DAccessorFunction &&accessor2d) const
{
    std::lock_guard lGuard{rwMutex};
    if(!accessor2d)
    {
        return ;
    }
    accessor2d(frame2D);
}

bool CygLidarD1::failedToRead() const
{
    return failureInRead.load();
}

} // namespace lidar_viewer::dev
