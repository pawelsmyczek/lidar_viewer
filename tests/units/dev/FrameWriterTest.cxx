#include "lidar_viewer/dev/FrameWriter.h"
#include "lidar_viewer/dev/IoStream.h"
#include "lidar_viewer/dev/IoStreamBase.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <future>
#include <chrono>

namespace lidar_viewer::tests::units
{

class MockFrameProvider {
public:
    struct MockFrame {
        std::vector<uint8_t> rawData;

        const uint8_t* raw() const { return rawData.data(); }
        size_t rawSize() const { return rawData.size(); }
    };

    MOCK_METHOD(void, use3dFrame, (std::function<void(const MockFrame&)>), (const));
};

class MockIoStream
: public dev::IoStreamBase
{
public:
    MockIoStream() = default;
    ~MockIoStream() noexcept override = default;

    MOCK_METHOD(void, open, (), (override, noexcept(false)));
    MOCK_METHOD(unsigned int, read, (void* ptr, unsigned int size, const std::chrono::milliseconds), ( const, override, noexcept(false)));
    MOCK_METHOD(void, write, (const void* ptr, unsigned int size, bool discardOutput), ( const, override ));
    MOCK_METHOD(void, close, (),  (const, override));
};

class FrameWriterTest : public ::testing::Test {
protected:
    MockFrameProvider mockFrameProvider;
    std::unique_ptr<MockIoStream> mockIoStream;
    dev::IoStream ioStream;
    lidar_viewer::dev::FrameWriter<MockFrameProvider> frameWriter;
    FrameWriterTest()
    : mockFrameProvider{}
    , mockIoStream{std::make_unique<MockIoStream>()}
    , ioStream{std::move(mockIoStream)}
    , frameWriter{mockFrameProvider, ioStream}
    {}

    ~FrameWriterTest()
    {
        frameWriter.stop();
    }
};

TEST_F(FrameWriterTest, Start_CreatesBackgroundThread)
{
    using namespace ::testing;
    EXPECT_CALL(mockFrameProvider, use3dFrame(_)).Times(AtLeast(1));

    frameWriter.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Give some time for thread execution
    frameWriter.stop();
}

TEST(FrameWriterStandalone, WritesFrameData)
{
    using namespace ::testing;
    MockFrameProvider mockFrameProvider;
    MockFrameProvider::MockFrame mockFrame;
    mockFrame.rawData = {1, 2, 3, 4};

    auto mockIoStream = std::make_unique<MockIoStream>();

    // Expect that write() is called with the correct data
    EXPECT_CALL(*mockIoStream, write(mockFrame.raw(), mockFrame.rawSize(), false))
            .Times(AtLeast(1));

    dev::IoStream ioStream{std::move(mockIoStream)};
    lidar_viewer::dev::FrameWriter<MockFrameProvider> frameWriter{mockFrameProvider, ioStream};


    // Simulate frame processing
    EXPECT_CALL(mockFrameProvider, use3dFrame(_))
            .WillRepeatedly(Invoke([&](std::function<void(const MockFrameProvider::MockFrame&)> callback) {
                callback(mockFrame);
            }));

    frameWriter.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    frameWriter.stop();
}

// Test exception handling in the background thread
TEST_F(FrameWriterTest, HandlesExceptionsGracefully)
{
    using namespace ::testing;

    EXPECT_CALL(mockFrameProvider, use3dFrame(_))
            .WillRepeatedly(Throw(std::runtime_error("Mock error")));

    EXPECT_NO_THROW({
                        frameWriter.start();
                        std::this_thread::sleep_for(std::chrono::milliseconds(20));
                        frameWriter.stop();
                    });
}

} // namespace lidar_viewer::tests::units

