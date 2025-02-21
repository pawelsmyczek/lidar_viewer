#ifndef LIDAR_VIEWER_CYGLIDARD1_H
#define LIDAR_VIEWER_CYGLIDARD1_H

#include "lidar_viewer/dev/IoStream.h"
#include "lidar_viewer/dev/CygLidarFrame.h"

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <future>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace lidar_viewer::dev
{

using Req2 = lidar_viewer::dev::Frame<2u>;
using Req3 = lidar_viewer::dev::Frame<3u>;

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

    static constexpr auto FRAME_SIZE_3D = static_cast<unsigned int>( 9600.f * 1.5f ) + 1u;
    static constexpr auto FRAME_SIZE_2D = 322u + 1u;

    using Frame3D = Frame < FRAME_SIZE_3D >;
    using Frame2D = Frame < FRAME_SIZE_2D >;

    using PointCloud3D = std::array<uint16_t , 160u * 60u>;
    using PointCloud2D = std::array<uint16_t , 160u>;

    /// accessor function for 3d point cloud
    using PointCloud3DAccessorFunction = std::function<void( const PointCloud3D&)>;

    /// accessor function for 3d point cloud
    template <typename ...Args>
    using PointCloud3DAccessorFunctionWithArgs = std::function<void( const PointCloud3D&, Args&... )>;

    /// accessor function for 3d point cloud
    using PointCloud2DAccessorFunction = std::function<void( const PointCloud2D& )>;

    /// accessor function for 3d frame
    using Frame3DAccessorFunction = std::function<void(const Frame3D& )>;
    /// accessor function for 2d frame
    using Frame2DAccessorFunction = std::function<void(const Frame2D& )>;

    /// modifier function for 3d point cloud
    using PointCloud3DModifierFunction = std::function<void( PointCloud3D& )>;
    /// modifier function for 3d point cloud
    using PointCloud2DModifierFunction = std::function<void( PointCloud2D& )>;

    /// ctor
    /// @param ioStream i/o stream representing lidar
    explicit CygLidarD1(IoStream& ioStream);

    ///  prints device info, firmware / hardware revision
    void printDeviceInfo();

    /// configures device according to definition of struct Config
    void configure(const Config cfg);

    /// sends a measurements receival start command according to mode
    /// @param mode take a look into Mode definition
    void run(Mode mode = Mode::Mode2D);

    /// stop measurement receival command
    void stop();

    const std::array<uint16_t , 160u * 60u>* get3dFrame() const;

    static constexpr std::pair<unsigned int, unsigned int> get3dFrameWindow() noexcept
    {
        return { 160u, 60u };
    }

    /// uses the 3D point cloud without modifying it, atomic access
    /// @param accessor3d function to access the 3d structure
    void use3dFrame(Frame3DAccessorFunction&& accessor3d) const;

    /// uses the 2D point cloud without modifying it, atomic access
    /// @param accessor2d function to access the 2d structure
    void use2dFrame(Frame2DAccessorFunction&& accessor2d) const;

    /// uses the 3D point cloud without modifying it, atomic access
    /// @param accessor3d function to access the 3d structure
    void use3dPointCloud(PointCloud3DAccessorFunction&& accessor3d) const;

    /// uses the 3D point cloud without modifying it, atomic access
    /// @param accessor3d function to access the 3d structure
    template <typename ... Args>
    void use3dPointCloudWithArgs(PointCloud3DAccessorFunctionWithArgs<Args&...> accessor3d, Args&... args) const
    {
        std::lock_guard lGuard{rwMutex};
        if (!accessor3d)
        {
            return;
        }
        accessor3d(pointcloud3d, args...);
    }

    /// uses the 2D point cloud without modifying it, atomic access
    /// @param accessor2d function to access the 2d structure
    void use2dPointCloud(PointCloud2DAccessorFunction&& accessor2d) const;

    void readAndParse3dFrame();
    void readAndParse2dFrame();

    bool failedToRead() const;

    CygLidarD1(const CygLidarD1&) = delete;
    CygLidarD1& operator = (const CygLidarD1&) = delete;
    CygLidarD1(CygLidarD1&&) = delete;
    CygLidarD1& operator = (CygLidarD1&&) = delete;

    /// dtor, disables communication with lidar
    ~CygLidarD1() noexcept;

private:

    template < typename Frame, typename ParsingFunction, typename TargetPointCloud>
    void readAndParseFrame( Frame& returnedFrame, ParsingFunction&& parsingFunction, TargetPointCloud& targetPointCloud)
    {
        try
        {
            const auto status = read(ioStream, returnedFrame);
            if(status == Status::BAD)
            {
                return ;
            }
            const auto returnedPayload = returnedFrame.payload();
            parsingFunction(targetPointCloud, returnedPayload);
        }
        catch (std::exception const & e)
        {
            using namespace std::string_literals;
            failureInRead = true;
            throw std::runtime_error{"Exception in read and parse : \n"s + e.what()};
        }
    }

    IoStream& ioStream;
    Frame < FRAME_SIZE_3D > frame3D;
    Frame < FRAME_SIZE_2D > frame2D;
    PointCloud3D pointcloud3d;
    PointCloud2D pointcloud2d;
    std::atomic<bool> failureInRead;
    mutable std::mutex rwMutex;
    mutable std::mutex pcMutex;
};

} // namespace lidar_viewer::dev
#endif // LIDAR_VIEWER_CYGLIDARD1_H
