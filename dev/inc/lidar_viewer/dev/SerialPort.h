#ifndef LIDAR_VIEWER_SERIALPORT_H
#define LIDAR_VIEWER_SERIALPORT_H

#include <chrono>
#include <string>

namespace lidar_viewer::dev
{

/// @brief simple RAII based serial port controller for unix stream
class SerialPort
{
public:
    explicit SerialPort(const std::string& fileName);
    ~SerialPort() noexcept;

    /// @brief wrapper around system open function
    [[nodiscard]] int open(const std::string& fileName) const;

    /// @brief configures serial port, sets 8N1 type of communication TODO make it configurable
    /// @param baudRate serial port baud rate, can be of any type, but takes into account regular types defined in termios-baud.h
    void configure(const unsigned int baudRate) const;

  /// @brief tries to read data from serial port up until a certain timeout
  /// expires
  /// @param ptr pointer to data to read from
  /// @param size size of data to read
  /// @param millis timeout
  unsigned int read(void *ptr, unsigned int size,
                    const std::chrono::milliseconds millis =
                        std::chrono::milliseconds::max()) const;

    /// @brief write data to serial port when available
    /// @param ptr pointer to data to write
    /// @param size size of data to write
    void write(const void* ptr, unsigned int size) const;

    /// @brief closes stream
    void close() const noexcept;
private:

    int fd;
};

} // namespace lidar_viewer::dev

#endif // LIDAR_VIEWER_SERIALPORT_H
