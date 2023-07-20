#ifndef LIDAR_VIEWER_CUSTOMBAUDRATESETTER_H
#define LIDAR_VIEWER_CUSTOMBAUDRATESETTER_H

namespace lidar_viewer::dev {

/// @brief sets custom baudrates (not available in termios-baud.h) to a tty
/// @param fd a file descriptor
/// @param baudRate baudrate to be set
void setCustomBaudrate(const int fd, const unsigned int baudRate);
}

#endif // LIDAR_VIEWER_CUSTOMBAUDRATESETTER_H
