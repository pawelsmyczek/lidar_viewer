#ifndef LIDAR_VIEWER_POINTCLOUDREADER_H
#define LIDAR_VIEWER_POINTCLOUDREADER_H

#include "CygLidarD1.h"

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

namespace lidar_viewer::dev
{

template < class PointCloudProvider >
class PointCloudReader
{
public:

    explicit PointCloudReader(PointCloudProvider &lidar)
            : lidar{lidar}
            , stopThread{false}
            , rxFuture{}
    { }

    ~PointCloudReader() noexcept
    {
        stop();
    }

    /// start measurement receival thread
    void start(PointCloudProvider::Mode mode)
    {
        using namespace std::string_literals;
        using namespace std::chrono_literals;
        rxFuture = std::async([this, mode]()
          {
              try
              {
                  for( ; !stopThread.load() ; )
                  {
                      mode == PointCloudProvider::Mode::Mode3D ? lidar.readAndParse3dFrame()
                                       : mode == PointCloudProvider::Mode::Mode2D ? lidar.readAndParse2dFrame()
                                       : (void)mode; // TODO: dual mode
                      std::this_thread::sleep_for(2ms);
                  }
              }
              catch (std::exception const & e)
              {
                  std::stringstream sstream{};
                  std::cerr << "Exception in point cloud reader operation: \n"s << e.what() << '\n';
              }
              catch (...)
              {
                  std::cerr << "Unknown exception in point cloud reader operation\n";
              }
          });
    }


    /// stop measurement receival thread
    void stop()
    {
        if ( stopThread.load() )
        {
            return ;
        }
        stopThread.store(true);
        if (rxFuture.valid())
        {
            rxFuture.wait();
        }
    }

    PointCloudReader(const PointCloudReader&) = delete;
    PointCloudReader& operator = (const PointCloudReader&) = delete;
    PointCloudReader(PointCloudReader&&) = delete;
    PointCloudReader& operator = (PointCloudReader&&) = delete;

private:

    PointCloudProvider& lidar;
    std::atomic<bool> stopThread;
    std::future<void> rxFuture;
};

} // namespace lidar_viewer::dev

#endif //LIDAR_VIEWER_POINTCLOUDREADER_H
