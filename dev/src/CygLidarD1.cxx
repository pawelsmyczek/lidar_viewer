#include "CygLidarD1.h"
#include "CygLidarFrame.h"
#include "StatusOr.h"

#include <csignal>
#include <termios.h>

#include <chrono>
#include <iostream>
#include <stdexcept>

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

lidar_viewer::dev::CygLidarD1* cygLidarD1Ptr = nullptr;

void handleSigTerm(int signo)
{
    if (signo == SIGTERM)
    {
        cygLidarD1Ptr->stop();
        ::exit(SIGTERM);
    }
}

void handleSigInt(int signo)
{
    if (signo == SIGINT)
    {
        cygLidarD1Ptr->stop();
        ::exit(SIGINT);
    }
}

}

namespace lidar_viewer::dev
{

CygLidarD1::PulseDuration::PulseDuration(
        PulseMode pulseMode, uint16_t duration_)
        : duration{duration_}
{
    duration |= static_cast<uint16_t>(pulseMode);
}

uint16_t CygLidarD1::PulseDuration::get() const
{
    return duration;
}

CygLidarD1::CygLidarD1(const std::string&  deviceName)
: serial{deviceName}
, pointcloud3d{}
, stopThread{false}
{
    if(!cygLidarD1Ptr)
    {
        cygLidarD1Ptr = this;
    }

    ::signal(SIGTERM, handleSigTerm);
    ::signal(SIGINT, handleSigInt);
}

CygLidarD1::~CygLidarD1()
{
    stop();
}

bool CygLidarD1::connected()
{
    using Req = Frame<2u>;
    using Resp = Frame<7u>;
    write(Req({RequestTypes::DeviceInfo, 0x00u}), serial);

    auto ret = read<Resp>(serial);
    if (ret.ok())
    {
        auto& retPayload = *ret.payload();
        static constexpr Resp::Payload expectedPayload{RequestTypes::DeviceInfo, 0x00u, 0x03u, 0x05u, 0x00u, 0x02, 0x02};
        return expectedPayload == retPayload;
    }
    return false;
}

void CygLidarD1::ioconfigure(const BaudRate baudRate) const
{
    const auto speedForSerial = baudRate == BaudRate::B57k6 ? B57600:
                       baudRate == BaudRate::B115k2 ? B115200:
                       baudRate == BaudRate::B250k ? 250000:
                       B3000000 ;
    serial.configure(speedForSerial);
}

void CygLidarD1::configure(const Config cfg)
{
    write( Frame<2u>({RequestTypes::BaudRate, static_cast<uint8_t>(cfg.baudRate)} ), serial );

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

    rxFuture = std::async([this]()
    {
        try
        {
            for( ; !stopThread ; )
            {
                read3dFrame();
#if 0
                const auto frameResolution = getFrameWindow();
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
        }
    });
}

void CygLidarD1::read3dFrame()
{
    using namespace std::chrono_literals;

    const auto ret = read< Frame<14401u> >(serial);
    if (!ret.ok())
    {
        return;
    }
    const auto returnedPayload = ret.payload();
    {
        std::lock_guard lGuard{rwMutex};
        auto frame3dIt = pointcloud3d.begin();
        auto returnedFrameIt = (returnedPayload->begin())+1;
        for ( ; frame3dIt != pointcloud3d.end() && returnedFrameIt != returnedPayload->end();
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
    }
    std::this_thread::sleep_for(67ms); // 15Hz timeout
}

const std::array<uint16_t, 9600u/*160x60*/> *CygLidarD1::get3dFrame() const
{
    std::lock_guard lGuard{rwMutex};
    return &pointcloud3d;
}

void CygLidarD1::stop()
{
    if ( stopThread )
    {
        return ;
    }
    write(Frame<2u>( {0x02, 0x00u} ),serial);
    stopThread = true;
    if (rxFuture.valid())
    {
        rxFuture.wait();
    }

    serial.close();
}

} // namespace lidar_viewer::dev
