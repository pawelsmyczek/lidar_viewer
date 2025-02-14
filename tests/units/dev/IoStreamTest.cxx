#include "lidar_viewer/dev/IoStream.h"
#include "lidar_viewer/dev/IoStreamBase.h"

#include <array>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace lidar_viewer::tests::units
{

struct IoStreamMock
: public dev::IoStreamBase
{
    IoStreamMock() = default;
    ~IoStreamMock() noexcept override = default;

    MOCK_METHOD(void, open, (), (override, noexcept(false)));
    MOCK_METHOD(unsigned int, read, (void* ptr, unsigned int size, const std::chrono::milliseconds), ( const, override, noexcept(false)));
    MOCK_METHOD(void, write, (const void* ptr, unsigned int size, bool discardOutput), ( const, override ));
    MOCK_METHOD(void, close, (),  (const, override));
};


struct IoStreamMockTestOpenAndCreate
        : public dev::IoStreamBase
{
    IoStreamMockTestOpenAndCreate()
    {
        EXPECT_CALL(*this, open());
        EXPECT_CALL(*this, close());
    }

    ~IoStreamMockTestOpenAndCreate() noexcept override = default;

    MOCK_METHOD(void, open, (), (override, noexcept(false)));
    MOCK_METHOD(unsigned int, read, (void* ptr, unsigned int size, const std::chrono::milliseconds), ( const, override, noexcept(false)));
    MOCK_METHOD(void, write, (const void* ptr, unsigned int size, bool discardOutput), ( const, override ));
    MOCK_METHOD(void, close, (),  (const, override));
};

TEST(TestIoStream, TestOpenIface)
{
    using namespace lidar_viewer::dev;
    auto dummyStream = std::make_unique<IoStreamMock>();
    EXPECT_CALL(*dummyStream, open());
    EXPECT_CALL(*dummyStream, close());
    IoStream ioStream{std::move(dummyStream)};
    ioStream.open();
}

TEST(TestIoStream, TestNoCloseCallIfIoStreamBaseIsNullptr)
{
    using namespace lidar_viewer::dev;
    auto dummyStream = std::make_unique<IoStreamMock>();
    EXPECT_CALL(*dummyStream, close()).Times(0);
    IoStream ioStream(std::unique_ptr<IoStreamBase>{nullptr});
}

TEST(TestIoStream, TestOpenAndCreateIface)
{
    using namespace lidar_viewer::dev;
    IoStream ioStream{};
    ioStream.createAndOpen<IoStreamMockTestOpenAndCreate>();
}

TEST(TestIoStream, TestWrite)
{
    using testing::_;
    using namespace lidar_viewer::dev;
    int dummyValue{2137};
    auto dummyStream = std::make_unique<IoStreamMock>();
    EXPECT_CALL(*dummyStream, close());
    EXPECT_CALL(*dummyStream, write(&dummyValue, sizeof(dummyValue), _))
        .WillOnce([&dummyValue](const void* ptr, unsigned int size, bool )
        {
            ASSERT_EQ(ptr, &dummyValue);
            ASSERT_EQ(size, sizeof(dummyValue));
        });
    IoStream ioStream{std::move(dummyStream)};
    ioStream.write(&dummyValue, sizeof(dummyValue));
}

TEST(TestIoStream, TestRead)
{
    using testing::_;
    using namespace lidar_viewer::dev;
    int dummyValueExpected{};
    auto dummyStream = std::make_unique<IoStreamMock>();
    EXPECT_CALL(*dummyStream, close());
    EXPECT_CALL(*dummyStream, read(&dummyValueExpected, sizeof(dummyValueExpected), _))
        .WillOnce([](void* ptr, unsigned int size, std::chrono::milliseconds )-> unsigned int
        {
            int dummyValue{2137};
            auto dummyValuePtr = reinterpret_cast<uint8_t*>(&dummyValue);
            auto inPtr = reinterpret_cast<uint8_t*>(ptr);
            for(auto i = 0u; i < size; i++)
            {
                inPtr[i] = dummyValuePtr[i];
            }
            return size;
        });
    IoStream ioStream{std::move(dummyStream)};
    ASSERT_EQ(sizeof(dummyValueExpected), ioStream.read(&dummyValueExpected, sizeof(dummyValueExpected), std::chrono::milliseconds{} ));
    ASSERT_EQ(dummyValueExpected, 2137);
}

} // namespace lidar_viewer::tests::units