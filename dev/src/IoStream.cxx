#include "lidar_viewer/dev/IoStream.h"

namespace lidar_viewer::dev
{

unsigned int IoStream::read(void *ptr, unsigned int size, const std::chrono::milliseconds millis) const noexcept(false)
{
    return ioStreamBase->read(ptr, size, millis);
}

void IoStream::write(const void *ptr, unsigned int size, bool discardOutput) const
{
    ioStreamBase->write(ptr, size, discardOutput);
}

void IoStream::close() const
{
    ioStreamBase->close();
}

IoStream::~IoStream() noexcept
{
    try
    {
        ioStreamBase->close();
    }
    catch ( ... )
    { /*NOOP*/ }
}

} // namespace dev::lidar_viewer


