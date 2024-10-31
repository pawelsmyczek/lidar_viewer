#ifndef LIDAR_VIEWER_CYGLIDARD1_H
#define LIDAR_VIEWER_CYGLIDARD1_H

#include "SerialPort.h"

#include <array>
#include <cstdint>
#include <future>
#include <mutex>
#include <thread>

namespace lidar_viewer::dev {

class CygLidarD1 {
  CygLidarD1(const CygLidarD1 &) = delete;
  CygLidarD1 &operator=(const CygLidarD1 &) = delete;

public:
  enum class Mode { Mode2D = 0x01u, Mode3D = 0x08u, Dual = 0x07u };

  enum class BaudRate {
    B57k6 = 0x39u,
    B115k2 = 0xaau,
    B250k = 0x77u,
    B3M = 0x55u
  };

  using FrequencyChannel = uint8_t;

  class PulseDuration {
  public:
    enum class PulseMode : uint16_t {
      Auto3D = 0u << 14u,
      Fixed3D = 1u << 14u,
      AutoDual,
      FixedDual
    };

    PulseDuration(PulseMode pulseMode, uint16_t duration_);

    uint16_t get() const;

  private:
    uint16_t duration;
  };

  struct Config {
    BaudRate baudRate;
    FrequencyChannel frequencyCh;
    PulseDuration pulseDuration;
    uint8_t sensitivity;
  };

  /// ctor, opens commnunication with lidar
  /// @param deviceName unix file name
  explicit CygLidarD1(const std::string &deviceName);

  /// dtor, disables communication with lidar
  ~CygLidarD1();

  /// configures io with baudrate
  void ioconfigure(const BaudRate baudRate) const;

  ///  checks if the device is connected
  bool connected();

  /// configures device according to definition of struct Config
  void configure(const Config cfg);

  /// run a measurements receival according to mode
  /// @param mode, take a look into Mode definition
  void run(Mode mode = Mode::Mode2D);

  /// start measurement receival thread
  void start(Mode mode = Mode::Mode2D);

  /// stop measurement receival thread
  void stop();

  const std::array<uint16_t, 9600u /*160x60*/> *get3dFrame() const;

  static constexpr std::pair<unsigned int, unsigned int>
  getFrameWindow() noexcept(true) {
    return {160u, 60u};
  }

private:
  void read3dFrame();

  SerialPort serial;

  std::array<uint16_t, 9600u /*160x60*/> pointcloud3d;
  std::atomic<bool> stopThread;
  std::future<void> rxFuture;
  mutable std::mutex rwMutex;
};

} // namespace lidar_viewer::dev
#endif // LIDAR_VIEWER_CYGLIDARD1_H
