#include "lidar_viewer/dev/CygLidarD1.h"
#include "lidar_viewer/dev/CygLidarFrame.h"

#include <chrono>
#include <iostream>
#include <stdexcept>

using namespace std::chrono_literals;

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

CygLidarD1::CygLidarD1(SerialPort& _serial)
: serial{_serial}
, pointcloud3d{}
, pointcloud2d{}
, stopThread{false}
{ }

CygLidarD1::~CygLidarD1() noexcept
{
    stop();
}

std::ostream& operator << (std::ostream& outStream, const Frame<7>& deviceInfo)
{
    for ( const auto value: *deviceInfo.payload() )
    {
        outStream << value << " ";
    }
    return outStream;
}

void CygLidarD1::printDeviceInfo()
{
    using Req = Frame<2u>;
    using Resp = Frame<7u>;
    write(Req({RequestTypes::DeviceInfo, 0x00u}), serial);

    auto ret = read<Resp>(serial);
    if (ret.ok())
    {
        auto& retPayload = *ret.payload();
        std::cout << "Device info: " << ret << "\n";
    }
    return ;
}

void CygLidarD1::configure(const Config cfg)
{
    write( Frame<2u>({RequestTypes::BaudRate, static_cast<uint8_t>(cfg.baudRate)} ), serial );
    std::this_thread::sleep_for(3s);
    const auto pulseDuration = cfg.pulseDuration.get();
    write( Frame<3u>({RequestTypes::SetLightPulseDuration, static_cast<uint8_t>(pulseDuration & 0xffu),
            static_cast<uint8_t>((pulseDuration & 0xff00u) >> 8u)} ), serial );
    write( Frame<2u>({RequestTypes::SetFreqChannel, static_cast<uint8_t>(cfg.frequencyCh)} ), serial );
    write( Frame<2u>({RequestTypes::SetSensitivity, static_cast<uint8_t>(cfg.sensitivity)} ), serial );
}

void CygLidarD1::run(Mode mode)
{
    write(Frame<2u>( {static_cast<uint8_t>(mode), 0x00u} ), serial);
}

void CygLidarD1::start(Mode mode)
{
    run( mode );

    rxFuture = std::async([this, mode]()
    {
        try
        {
            for( ; !stopThread.load() ; )
            {
                mode == Mode::Mode3D ? readAndParse3dFrame()
                : mode == Mode::Mode2D ? readAndParse2dFrame()
                : (void)mode; // TODO: dual mode
#if 0
                const auto frameResolution = get3dFrameWindow();
                unsigned int frameId = 0u;
                for ( unsigned int y = 0; y < frameResolution.second; ++y )
                {
                    for ( unsigned int x = 0; x < frameResolution.first; ++x )
                    {
                        frameId = y*frameResolution.first + x;
                        std::cout << frame3d[frameId] << (x % (frameResolution.first) == 0? "\n": " ");
                    }
                }
                std::cout  << "\n";
#endif
            }
        } catch (const std::runtime_error& e)
        {
            stop();
            std::cerr << "Failure in lidar operation: " << e.what() << "\n";
            throw ;
        }
    });
}

void CygLidarD1::readAndParse3dFrame()
{
    static constexpr auto FRAME_SIZE = static_cast<unsigned int>( 9600.f * 1.5f ) + 1u;
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
    readAndParseFrame<FRAME_SIZE>(parsingFunction, pointcloud3d);
}

void CygLidarD1::readAndParse2dFrame()
{
    static constexpr auto FRAME_SIZE = 322u + 1u;
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
    readAndParseFrame<FRAME_SIZE>(parsingFunction, pointcloud2d);
}

const std::array<uint16_t, 160u*60u>* CygLidarD1::get3dFrame() const
{
    std::lock_guard lGuard{rwMutex};
    return &pointcloud3d;
}

void CygLidarD1::stop()
{
    if ( stopThread.load() )
    {
        return ;
    }
    write(Frame<2u>( {0x02, 0x00u} ),serial);
    stopThread.store(true);
    if (rxFuture.valid())
    {
        rxFuture.wait();
    }
}

} // namespace lidar_viewer::dev
