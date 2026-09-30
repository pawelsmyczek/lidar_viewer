#include "lidar_viewer/dev/CygLidarD1.h"
#include "lidar_viewer/dev/IoStream.h"
#include "lidar_viewer/dev/IoStreamBase.h"
#include "lidar_viewer/ui/display/DisplayFlatDepthImage.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using ::testing::Return;
using ::testing::Invoke;

namespace lidar_viewer::tests::units
{

class MockCygLidarD1 : public dev::CygLidarD1
{
public:
    MockCygLidarD1(dev::IoStream& ioStream)
    : dev::CygLidarD1(ioStream)
    {}

    MOCK_METHOD(bool, failedToRead, (), (const, override));
    MOCK_METHOD(void, use3dPointCloud, (PointCloud3DAccessorFunction&& accessor3d), (const, override));
};

struct MockDrawPoint
{
    MockDrawPoint() = default;

    MOCK_METHOD(void, operatorCall, (const geometry::types::Point3D<float>&, (const std::array<uint8_t, 3>&)), (const));

    void operator()(const geometry::types::Point3D<float>& point, const std::array<uint8_t, 3>& color) const
    {
        operatorCall(point, color);
    }
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

TEST(DisplayFlatDepthImageTest, ReturnsFalseIfLidarIsNull)
{
    MockDrawPoint drawPoint{};
    const auto ptr = nullptr;
    EXPECT_FALSE(ui::displayFlatDepthImage(ptr, std::cref(drawPoint)));
}

TEST(DisplayFlatDepthImageTest, ReturnsFalseIfLidarFailsToRead)
{
    auto mockIoStream = std::make_unique<MockIoStream>();
    dev::IoStream ioStream{std::move(mockIoStream)};
    MockCygLidarD1 mockLidar{ioStream};
    MockDrawPoint drawPoint;

    EXPECT_CALL(mockLidar, failedToRead()).WillOnce(Return(true));

    EXPECT_FALSE(ui::displayFlatDepthImage(&mockLidar, std::cref(drawPoint)));
}

TEST(DisplayFlatDepthImageTest, ProcessesValidPointCloud)
{
    using ::testing::_;
    auto mockIoStream = std::make_unique<MockIoStream>();
    dev::IoStream ioStream{std::move(mockIoStream)};
    MockCygLidarD1 mockLidar{ioStream};
    MockDrawPoint drawPoint;

    EXPECT_CALL(mockLidar, failedToRead()).WillOnce(Return(false));

    std::array<uint16_t, 160*60> mockPointCloud{};
    // Simulate valid depth points
    std::fill(mockPointCloud.begin(), mockPointCloud.end(), 1000);
    EXPECT_CALL(mockLidar, use3dPointCloud(_))
            .WillOnce(Invoke([&](auto func) { func(mockPointCloud); }));

    EXPECT_CALL(drawPoint, operatorCall(_, _)).Times(::testing::AtLeast(1));

    EXPECT_TRUE(ui::displayFlatDepthImage(&mockLidar, std::cref(drawPoint)));
}

TEST(DisplayFlatDepthImageTest, SkipsInvalidDepthValues)
{
    using ::testing::_;
    auto mockIoStream = std::make_unique<MockIoStream>();
    dev::IoStream ioStream{std::move(mockIoStream)};
    MockCygLidarD1 mockLidar{ioStream};
    MockDrawPoint drawPoint;

    EXPECT_CALL(mockLidar, failedToRead()).WillOnce(Return(false));

    std::array<uint16_t, 160*60> mockPointCloud{};
    // Simulate aout of range depth points
    std::fill(mockPointCloud.begin(), mockPointCloud.end(), 5000);
    EXPECT_CALL(mockLidar, use3dPointCloud(_))
            .WillOnce(Invoke([&](auto func) { func(mockPointCloud); }));

    EXPECT_CALL(drawPoint, operatorCall(_, _)).Times(0); // No calls expected

    EXPECT_TRUE(ui::displayFlatDepthImage(&mockLidar, std::cref(drawPoint)));
}
} // namespace lidar_viewer::tests::units

