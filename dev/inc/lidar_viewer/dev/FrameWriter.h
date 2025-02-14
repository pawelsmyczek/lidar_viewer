#ifndef LIDAR_VIEWER_FRAMEWRITER_H
#define LIDAR_VIEWER_FRAMEWRITER_H

#include "CygLidarD1.h"

#include <atomic>
#include <thread>

namespace lidar_viewer::dev
{

template <class FrameProvider>
class FrameWriter
{
public:

    /// @brief ctor
    /// @param lidar reference to lidar
    /// @param ioStream reference to ioStream
    explicit FrameWriter(const FrameProvider& lidar, IoStream& ioStream);
    ~FrameWriter() noexcept;

    /// start writer thread
    void start();

    /// stop measurement receival thread
    void stop();

    FrameWriter(const FrameWriter&) = delete;
    FrameWriter& operator = (const FrameWriter&) = delete;
    FrameWriter(FrameWriter&&) = delete;
    FrameWriter& operator = (FrameWriter&&) = delete;

private:

    const FrameProvider&   lidar;
    IoStream&           ioStream;
    std::atomic<bool>   stopThread;
    std::future<void>   rxFuture;
};

template <class FrameProvider>
FrameWriter<FrameProvider>::FrameWriter(const FrameProvider& _lidar, IoStream& _ioStream)
        : lidar{_lidar}
        , ioStream{_ioStream}
        , stopThread{false}
        , rxFuture{}
{ }

template <class FrameProvider>
FrameWriter<FrameProvider>::~FrameWriter() noexcept
{
    stop();
}

template <class FrameProvider>
void FrameWriter<FrameProvider>::start()
{
    rxFuture = std::async([this]()
      {
          using namespace std::chrono_literals;
          using namespace std::string_literals;
          try
          {
              for( ; !stopThread.load() ; )
              {
                  lidar.use3dFrame([this](const auto& frame)
                                   {
                                       ioStream.write(frame.raw(), frame.rawSize());
                                   });
                  std::this_thread::sleep_for(5ms);
              }
          } catch (const std::exception& e)
          {
              stop();
              std::cerr << "Failure in point cloud writer operation: "s + e.what() << '\n';
              throw ;
          }
      });
}

template <class FrameProvider>
void FrameWriter<FrameProvider>::stop()
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


} // namespace lidar_viewer::dev

#endif //LIDAR_VIEWER_FRAMEWRITER_H
