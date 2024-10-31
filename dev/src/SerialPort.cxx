#include "SerialPort.h"
#include "CustomBaudrateSetter.h"

#include <unistd.h>

#include <asm/ioctls.h>
#include <csignal>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>

#include <chrono>
#include <iostream>
#include <stdexcept>

namespace {

bool pollFor(int fd, const std::chrono::milliseconds ms,
             const int16_t pollType) noexcept(true) {
  using namespace std::string_literals;
  const auto count = ms.count();
  pollfd pfd{.fd = fd, .events = pollType};
  if (const auto ret = ::poll(&pfd, 1, count); !(pfd.revents & pollType)) {
    std::cerr << count << "ms timeout expired on "
              << (pollType == POLLIN ? "read" : "write")
              << ": revents: " << pfd.revents << "\n";
    return false;
  } else if (ret < 0 || pfd.revents & (POLLHUP | POLLERR)) {
    std::cerr << "::poll for " << (pollType == POLLIN ? "read" : "write")
              << " on fd: " << fd << " returned: " << ret
              << ", revents: " << pfd.revents << "\n";
    return false;
  }
  return true;
}

lidar_viewer::dev::SerialPort *serial = nullptr;

void handleSigTerm(int signo) {
  if (signo == SIGTERM) {
    serial->close();
    exit(SIGTERM);
  }
}

void handleSigInt(int signo) {
  if (signo == SIGINT) {
    serial->close();
    exit(SIGINT);
  }
}

} // namespace

namespace lidar_viewer::dev {

SerialPort::SerialPort(const std::string &fileName)
    : fd{::open(fileName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC)} {
  if (fd < 0) {
    throw std::runtime_error("failed to open stream: " + std::to_string(fd));
  }

  if (!serial) {
    serial = this;
  }

  ::signal(SIGTERM, handleSigTerm);
  ::signal(SIGINT, handleSigInt);
}

SerialPort::~SerialPort() { close(); }

void SerialPort::configure(const unsigned int baudRate) const {
  using namespace std::string_literals;
  termios ttynew{}, ttyold{};

  const auto standardBaud = baudRate >= B57600 && baudRate <= B4000000;
  if (!standardBaud) {
    setCustomBaudrate(fd, baudRate);
  }

  if (const auto ret = ::ioctl(fd, TCGETS, &ttyold); ret < 0) {
    throw std::runtime_error(__func__ + "::ioctl returned : "s +
                             std::to_string(ret));
  }
  if (standardBaud) {
    if (const auto ospeedret = ::cfsetospeed(&ttynew, baudRate);
        ospeedret < 0) {
      throw std::runtime_error(__func__ + "::cfsetospeed returned : "s +
                               std::to_string(ospeedret));
    }
    if (const auto ispeedret = ::cfsetispeed(&ttynew, baudRate);
        ispeedret < 0) {
      throw std::runtime_error(__func__ + "::cfsetispeed returned : "s +
                               std::to_string(ispeedret));
    }
  }

  ttynew.c_cc[VTIME] = 0;
  ttynew.c_cc[VMIN] = 0;
  ttynew.c_cflag |= (CREAD | CLOCAL);
  ttynew.c_cflag &= ~CSIZE;
  ttynew.c_cflag |= CS8;     /* 8-bit characters */
  ttynew.c_cflag &= ~PARENB; /* no parity bit */
  ttynew.c_cflag &= ~CSTOPB; /* only need 1 stop bit */

  ttynew.c_iflag &=
      ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  // no flow control
  ttynew.c_cflag &= ~CRTSCTS;
  ttynew.c_iflag &= ~(IXON | IXOFF | IXANY);

  // no canonical input & output
  ttynew.c_lflag &=
      ~(ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHOKE | ECHOCTL);
  ttynew.c_lflag |= ISIG;
  ttynew.c_oflag &= ~(OPOST | ONLCR);

  if (const auto ret = ::ioctl(fd, TCSETS, &ttynew); ret < 0) {
    throw std::runtime_error(__func__ + "::ioctl returned : "s +
                             std::to_string(ret));
  }
}

unsigned int SerialPort::read(void *ptr, unsigned int size,
                              const std::chrono::milliseconds millis) const {
  using namespace std::chrono_literals;
  using namespace std::string_literals;
  auto rret = 0;
  auto rretsum = 0u;
  auto *uptr = reinterpret_cast<uint8_t *>(ptr);
  do {
    if (!pollFor(fd, millis, POLLIN)) {
      continue;
    }

    uptr += rret;

    if (rret = static_cast<int>(::read(fd, uptr, size - rretsum)); rret <= 0) {
      throw std::runtime_error("::read returned: "s + std::to_string(rret));
    }
    rretsum += rret;
  } while (rretsum < size);
  return rretsum;
}

void SerialPort::write(const void *ptr, unsigned int size) const {
  using namespace std::chrono_literals;
  using namespace std::string_literals;

  pollFor(fd, 100ms, POLLOUT);

  if (const auto wret = ::write(fd, ptr, size); wret != size) {
    throw std::runtime_error(__func__ + ", ::write returned: "s +
                             std::to_string(wret));
  }
}

void SerialPort::close() const {
  if (fd > 0) {
    std::cout << "Closing fd: " << fd << "\n";
    ::close(fd);
  }
}

} // namespace lidar_viewer::dev