#ifndef LIDAR_VIEWER_CYGLIDARD1_H
#define LIDAR_VIEWER_CYGLIDARD1_H

#include "lidar_viewer/dev/SerialPort.h"
#include "lidar_viewer/dev/CygLidarFrame.h"

#include <atomic>
#include <array>
#include <cstdint>
#include <future>
#include <mutex>
#include <thread>

namespace lidar_viewer::dev
{

class CygLidarD1
{
public:

    enum class ErrorCodes3D: uint16_t
    {
        LowAmplitude    = 4081u,
        AdcOverflow     = 4082u,
        Saturation      = 4083u
    };

    enum class ErrorCodes2D: uint16_t
    {
        LowAmplitude    = 16001u,
        AdcOverflow     = 16002u,
        Saturation      = 16003u,
        BadPixel        = 16004u
    };

    enum class Mode: uint8_t
    {
        Mode2D  = 0x01u,
        Mode3D  = 0x08u,
        Dual    = 0x07u
    };

    enum class BaudRate: uint8_t
    {
        B57k6   = 0x39u,
        B115k2  = 0xaau,
        B250k   = 0x77u,
        B3M     = 0x55u
    };

    using FrequencyChannel = uint8_t;

    class PulseDuration
    {
    public:
        enum class PulseMode : uint16_t
        {
            Auto3D  = 0u << 14u,
            Fixed3D = 1u << 14u,
            AutoDual,
            FixedDual
        };

        PulseDuration(PulseMode pulseMode, uint16_t duration_);
        PulseDuration();

        [[nodiscard]] uint16_t get() const;

        PulseDuration(const PulseDuration&) = default;
        PulseDuration& operator = (const PulseDuration& ) = default;
    private:
        uint16_t duration;
    };

    struct Config
    {
        BaudRate baudRate;
        FrequencyChannel frequencyCh;
        PulseDuration pulseDuration;
        uint8_t sensitivity;
    };

    /// ctor, opens commnunication with lidar
    /// @param deviceName unix file name
    explicit CygLidarD1(SerialPort& _serial);

    /// configures io with baudrate
    void ioconfigure(const BaudRate baudRate) const;

    ///  prints device info, firmware / hardware revision
    void printDeviceInfo();

    /// configures device according to definition of struct Config
    void configure(const Config cfg);

    /// run a measurements receival according to mode
    /// @param mode, take a look into Mode definition
    void run(Mode mode = Mode::Mode2D);

    /// start measurement receival thread
    void start(Mode mode = Mode::Mode2D);

    /// stop measurement receival thread
    void stop();

    const std::array<uint16_t , 160u * 60u>* get3dFrame() const;

    static constexpr std::pair<unsigned int, unsigned int> get3dFrameWindow() noexcept
    {
        return { 160u, 60u };
    }

    CygLidarD1(const CygLidarD1&) = delete;
    CygLidarD1& operator = (const CygLidarD1&) = delete;

    /// dtor, disables communication with lidar
    ~CygLidarD1() noexcept;

private:

    template < uint16_t FRAME_SIZE, typename ParsingFunction, typename TargetPointCloud>
    void readAndParseFrame(ParsingFunction&& parsingFunction, TargetPointCloud& targetPointCloud)
    {
        static Frame < FRAME_SIZE > returnedFrame {};
        read(serial, returnedFrame);
        if(returnedFrame.ok() == StatusOr::Status::BAD)
        {
            return ;
        }
        const auto returnedPayload = returnedFrame.payload();

        std::lock_guard lGuard{rwMutex};
        parsingFunction(targetPointCloud, returnedPayload);
    }

    void readAndParse3dFrame();
    void readAndParse2dFrame();

    SerialPort serial;
    std::array<uint16_t , 160u * 60u> pointcloud3d;
    std::array<uint16_t , 160u> pointcloud2d;
    std::atomic<bool> stopThread;
    std::future<void> rxFuture;
    mutable std::mutex rwMutex;
};

} // namespace lidar_viewer::dev
#endif // LIDAR_VIEWER_CYGLIDARD1_H
