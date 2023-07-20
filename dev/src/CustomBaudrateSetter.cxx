#include "CustomBaudrateSetter.h"

#include <asm/termbits.h>
#include <sys/ioctl.h>

#include <stdexcept>

namespace lidar_viewer::dev
{

void setCustomBaudrate(const int fd, const unsigned int baudRate)
{
    termios2 tionew{}, tioold{}, tiocheck{};

    if(const auto ioret = ::ioctl(fd, TCGETS2, &tioold);
    ioret < 0)
    {
        throw std::runtime_error("Error from ::ioctl(... TCGETS2) returned: " + std::to_string(ioret));
    }

    tionew.c_cflag &= ~CBAUD;
    tionew.c_cflag |= BOTHER;
    tionew.c_ispeed = baudRate;
    tionew.c_ospeed = baudRate;
    if(const auto ioret = ::ioctl(fd, TCSETS2, &tionew);
            ioret < 0)
    {
        throw std::runtime_error("Error from ::ioctl(... TCSETS2) returned: " + std::to_string(ioret));
    }

    // verify

    if(const auto ioret = ::ioctl(fd, TCGETS2, &tiocheck);
            ioret < 0)
    {
        throw std::runtime_error("Error from ::ioctl(... TCGETS2) returned: " + std::to_string(ioret));
    }

    if (tionew.c_ospeed != baudRate || tionew.c_ispeed != baudRate)
    {
        // failure
        throw std::runtime_error("Failed to set baud rate to value: " + std::to_string(baudRate));
    }
}

}
