#include "lidar_viewer/dev/SerialPort.h"
#include "lidar_viewer/dev/CustomBaudrateSetter.h"

#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <asm/ioctls.h>
#include <cstring>

#include <chrono>
#include <stdexcept>
#include <iostream>

namespace
{

bool pollFor(int fd, const std::chrono::milliseconds ms, const int16_t pollType) noexcept(true)
{
    using namespace std::string_literals;
    const auto count = ms.count();
    pollfd pfd
            {
                    .fd = fd,
                    .events = pollType,
                    .revents{}
            };
    if(const auto ret = ::poll(&pfd, 1, static_cast<int>(count));
            !(pfd.revents & pollType))
    {
        std::cerr << count << "ms timeout expired on "<< (pollType == POLLIN? "read": "write") << ": revents: " << pfd.revents <<"\n";
        return false;
    }
    else if(ret < 0 || pfd.revents & (POLLHUP | POLLERR))
    {
        std::cerr << "::poll for " << (pollType == POLLIN? "read": "write") << " on fd: " << fd << " returned: "
                  << ret << ", revents: " << pfd.revents << "\n";
        return false;
    }
    return true ;
}

}

namespace lidar_viewer::dev
{

SerialPort::SerialPort(const std::string& fileName)
: fd{ open(fileName) }
{ }

SerialPort::SerialPort(const std::string &fileName, const unsigned int baudRate)
: fd{ open(fileName) }
{
    configure(baudRate);
}

SerialPort::~SerialPort() noexcept
{
    close();
}

int SerialPort::open(const std::string& fileName) const
{
    auto rret = ::open(fileName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if(rret < 0)
    {
        using namespace std::string_literals;
        throw std::runtime_error("failed to open stream: "s + ::strerror(fd));
    }
    return rret;
}

void SerialPort::configure(const unsigned int baudRate) const
{
    using namespace std::string_literals;
    termios ttynew{}, ttyold{};

    if (const auto standardBaud = baudRate >= B57600 && baudRate <= B4000000;
                                !standardBaud)
    {
        setCustomBaudrate(fd, baudRate);
    }
    else
    {
        if (const auto ret = ::ioctl (fd, TCGETS, &ttyold);
                ret < 0 )
        {
            throw std::runtime_error (__func__ + "::ioctl returned : "s + ::strerror(ret) );
        }

        if(const auto ospeedret = ::cfsetospeed (&ttynew, baudRate);
                ospeedret < 0)
        {
            throw std::runtime_error (__func__ + "::cfsetospeed returned : "s + ::strerror(ospeedret) );
        }
        if(const auto ispeedret = ::cfsetispeed (&ttynew, baudRate);
                ispeedret < 0)
        {
            throw std::runtime_error (__func__ + "::cfsetispeed returned : "s + ::strerror(ispeedret) );
        }
    }

    ttynew.c_cc[VTIME] = 0;
    ttynew.c_cc[VMIN] = 0;
    ttynew.c_cflag |= (CREAD | CLOCAL);
    ttynew.c_cflag &=  ~CSIZE;
    ttynew.c_cflag |= CS8;         /* 8-bit characters */
    ttynew.c_cflag &= ~PARENB;     /* no parity bit */
    ttynew.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */

    ttynew.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    // no flow control
    ttynew.c_cflag &= ~CRTSCTS;
    ttynew.c_iflag &= ~(IXON | IXOFF | IXANY);

    // no canonical input & output
    ttynew.c_lflag &= ~(ICANON | IEXTEN
                        | ECHO | ECHOE | ECHOK | ECHOKE | ECHOCTL);
    ttynew.c_lflag |= ISIG;
    ttynew.c_oflag &= ~(OPOST | ONLCR);

    if (const auto ret = ::ioctl (fd, TCSETS, &ttynew);
            ret < 0 )
    {
        throw std::runtime_error (__func__ + "::ioctl returned : "s + ::strerror(ret) );
    }

}

unsigned int SerialPort::read(void *ptr, unsigned int size, const std::chrono::milliseconds millis) const
{
    using namespace std::chrono_literals;
    using namespace std::string_literals;
    auto rret = 0;
    auto rretsum = 0u;
    auto uptr = reinterpret_cast<uint8_t*>(ptr);
    do
    {
        if( !pollFor(fd, millis, POLLIN))
        {
            return rretsum ;
        }

        uptr += rret;

        if( rret = static_cast<int>(::read(fd, uptr, size-rretsum));
                rret <= 0 )
        {
            throw std::runtime_error("::read returned: "s + ::strerror(rret) );
        }
        rretsum += rret;
    } while ( rretsum < size );
    return rretsum;
}

void SerialPort::write(const void *ptr, unsigned int size, bool ) const noexcept(false)
{
    using namespace std::chrono_literals;
    using namespace std::string_literals;

    pollFor(fd, 100ms, POLLOUT);

    if( const auto wret = ::write(fd, ptr, size);
            wret != size )
    {
        throw std::runtime_error(__func__ + ", ::write returned: "s + ::strerror(static_cast<int>(wret)) );
    }
}

void SerialPort::close() const noexcept
{
    using namespace std::string_literals;
    static auto fdClosed = false;
    if(fdClosed)
    {
        return ;
    }
    if(fd < 0)
    {
        std::cerr <<  "Closing an invalid file descriptor: "s;
        return ;
    }

   if (const auto cret = ::close(fd); cret < 0)
    {
        std::cerr <<  "::close returned: "s << ::strerror(cret);
        return ;
    }
    std::cout << "Closed fd: " << fd << "\n";
    fdClosed = !fdClosed;
}

void SerialPort::setFd(const int _fd)
{
    fd = _fd;
}

void SerialPort::open()
{

}

}
