#ifndef LIDAR_VIEWER_BINARYFILE_H
#define LIDAR_VIEWER_BINARYFILE_H

#include "IoStream.h"
#include <fstream>

namespace lidar_viewer::dev
{

class BinaryFile
        : public IoStreamBase
{
public:
    explicit BinaryFile(const std::string& fileName);
    ~BinaryFile() noexcept override = default;

    /// @brief tries to  an i/o stream
    void open() override;

    /// @brief tries to read data from binary file up until a certain timeout expires
    /// @param ptr pointer to data to read from
    /// @param size size of data to read
    /// @param millis timeout
    unsigned int read(void* ptr, unsigned int size, const std::chrono::milliseconds millis = std::chrono::milliseconds::max()) const noexcept(false) override;

    /// @brief write data to binary file when available
    /// @param ptr pointer to data to write
    /// @param size size of data to write
    void write(const void* ptr, unsigned int size, bool discardOutput = true) const override;

    /// @brief closes stream
    void close() const noexcept override;

    BinaryFile(const BinaryFile& ) = delete;
    BinaryFile& operator = (const BinaryFile& ) = delete;
    BinaryFile(BinaryFile&& ) = delete;
    BinaryFile& operator = (BinaryFile&& ) = delete;

private:
    mutable std::fstream fStream;
};

} // dev::lidar_viewer

#endif //LIDAR_VIEWER_BINARYFILE_H
