#include "lidar_viewer/dev/BinaryFile.h"
#include <iostream>
#include <thread>

namespace lidar_viewer::dev
{

BinaryFile::BinaryFile(const std::string &fileName)
: fStream{fileName, std::fstream::binary | std::fstream::in | std::fstream::out}
{
    if(!fStream.is_open())
    {
        throw std::runtime_error{ "Unable to open file : " + fileName };
    }
}

void BinaryFile::open()
{ /*NOOP*/ }

unsigned int BinaryFile::read(void *ptr, unsigned int size, const std::chrono::milliseconds ) const noexcept(false)
{
    if(fStream.eof())
    {
        throw std::runtime_error{"BinaryFile::read : reached EOF"};
    }
    fStream.read(reinterpret_cast<char*>(ptr), size);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    return size;
}

void BinaryFile::write(const void *ptr, unsigned int size, bool discardOutput) const
{
    if(discardOutput)
    {
        return ;
    }
    fStream.write(reinterpret_cast<const char*>(ptr), size);
    fStream.flush();
}

void BinaryFile::close() const noexcept
{ /*NOOP*/}

} // dev::lidar_viewer