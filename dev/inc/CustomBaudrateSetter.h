#ifndef LIDAR_VIEWER_CUSTOMBAUDRATESETTER_H
#define LIDAR_VIEWER_CUSTOMBAUDRATESETTER_H
#include "CygLidarD1.h"

namespace lidar_viewer::dev {

void setCustomBaudrate(const int fd, const unsigned int baudRate);
}

#endif // LIDAR_VIEWER_CUSTOMBAUDRATESETTER_H
