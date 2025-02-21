#include "lidar_viewer/dev/PointCloudReader.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace lidar_viewer::tests::units
{
// Mock class for PointCloudProvider
class MockPointCloudProvider {
public:
    enum class Mode { Mode2D, Mode3D };

    MOCK_METHOD(void, readAndParse3dFrame, (), ());
    MOCK_METHOD(void, readAndParse2dFrame, (), ());
};

struct PointCloudReaderTest
        : public ::testing::Test
{
    PointCloudReaderTest()
    : mockLidar{}
    , reader{mockLidar}
    {}

    ~PointCloudReaderTest()
    {
        reader.stop();
    }

protected:
    MockPointCloudProvider mockLidar;
    dev::PointCloudReader<MockPointCloudProvider> reader;
};

// Test: Ensures PointCloudReader starts correctly in 3D mode
TEST_F(PointCloudReaderTest, StartsAndCalls3DFrameParsing)
{
    EXPECT_CALL(mockLidar, readAndParse3dFrame())
    .Times(testing::AtLeast(1));

    reader.start(MockPointCloudProvider::Mode::Mode3D);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    reader.stop();
}

// Test: Ensures PointCloudReader starts correctly in 2D mode
TEST_F(PointCloudReaderTest, StartsAndCalls2DFrameParsing)
{
    EXPECT_CALL(mockLidar, readAndParse2dFrame())
    .Times(testing::AtLeast(1));

    reader.start(MockPointCloudProvider::Mode::Mode2D);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    reader.stop();
}

// Test: Ensures calling stop() multiple times does not cause issues
TEST_F(PointCloudReaderTest, StopIsIdempotent)
{
    EXPECT_CALL(mockLidar, readAndParse3dFrame()).Times(3);
    reader.start(MockPointCloudProvider::Mode::Mode3D);
    std::this_thread::sleep_for(std::chrono::milliseconds(6));
    reader.stop();
    EXPECT_NO_THROW(reader.stop());  // Calling stop again should not cause issues
}

// Test: Handles exceptions in thread gracefully
TEST_F(PointCloudReaderTest, HandlesExceptionsGracefully)
{
    EXPECT_CALL(mockLidar, readAndParse3dFrame())
    .WillOnce(testing::Throw(std::runtime_error("Lidar error!")));

    EXPECT_NO_THROW(reader.start(MockPointCloudProvider::Mode::Mode3D));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    reader.stop();
}
} // namespace lidar_viewer::tests::units