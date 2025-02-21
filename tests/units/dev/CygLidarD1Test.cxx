#include "lidar_viewer/dev/CygLidarD1.h"
#include "lidar_viewer/dev/IoStream.h"
#include "lidar_viewer/dev/IoStreamBase.h"
#include "lidar_viewer/dev/CygLidarFrame.h"

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

using namespace std::string_literals;

TEST(CygLidarD1Test, CygLidarD1DepthImageReadAndUseTestPositiveCase)
{
    using namespace lidar_viewer::dev;
    using testing::_;
    using testing::Sequence;
    using testing::Return;
    Frame < CygLidarD1::FRAME_SIZE_3D > dummyValueExpected{};
    auto sizeOfFrame = dummyValueExpected.size();
    auto header = dummyValueExpected.header();

    auto dummyStream = std::make_unique<IoStreamMock>();
    Sequence seq;

    // just a single call at the end of scope,
    // don't care about the configuration, just
    // like with regular file reading
    EXPECT_CALL(*dummyStream, write(_, _, _));
    EXPECT_CALL(*dummyStream, close());

    EXPECT_CALL(*dummyStream, read(_,
                                   dummyValueExpected.header().size(), _)).InSequence(seq)
                                   .WillOnce(
                                   [&header] (void* ptr, unsigned int , std::chrono::milliseconds )
                                   {
                                            auto ptrUint8 = reinterpret_cast<uint8_t*>(ptr);
                                            for(unsigned int i = 0; i < header.size(); ++i)
                                            {
                                                ptrUint8[i] = header[i];
                                            }
                                           return header.size();
                                   });
    EXPECT_CALL(*dummyStream, read(_,
                                   sizeof(decltype(sizeOfFrame)), _)).InSequence(seq)
                                    .WillOnce(
                                            [&sizeOfFrame] (void* ptr, unsigned int , std::chrono::milliseconds )
                                            {
                                                auto ptrUint8 = reinterpret_cast<uint8_t*>(ptr);
                                                auto sizeOfFrameUint8 = reinterpret_cast<uint8_t*>(&sizeOfFrame);

                                                for(unsigned int i = 0; i < sizeof(decltype(sizeOfFrame)); ++i)
                                                {
                                                    ptrUint8[i] = sizeOfFrameUint8[i];
                                                }
                                                return sizeof(decltype(sizeOfFrame));
                                            }
                                            );
    EXPECT_CALL(*dummyStream, read(_,
               dummyValueExpected.size()+1, _)).InSequence(seq)
                        .WillOnce(Return(dummyValueExpected.size()+1));

    IoStream ioStream{std::move(dummyStream)};
    CygLidarD1 lidar{ioStream};
    lidar.readAndParse3dFrame();
    lidar.use3dPointCloud([](const CygLidarD1::PointCloud3D& ) {});
}

TEST(CygLidarD1Test, CygLidarD12dFrameReadAndUseTestPositiveCase)
{
    using namespace lidar_viewer::dev;
    using testing::_;
    using testing::Sequence;
    using testing::Return;
    Frame < CygLidarD1::FRAME_SIZE_2D > dummyValueExpected{};
    auto sizeOfFrame = dummyValueExpected.size();
    auto header = dummyValueExpected.header();

    auto dummyStream = std::make_unique<IoStreamMock>();
    Sequence seq;

    // just a single call at the end of scope,
    // don't care about the configuration, just
    // like with regular file reading
    EXPECT_CALL(*dummyStream, write(_, _, _));
    EXPECT_CALL(*dummyStream, close());

    EXPECT_CALL(*dummyStream, read(_,
                                   dummyValueExpected.header().size(), _)).InSequence(seq)
            .WillOnce(
                    [&header] (void* ptr, unsigned int , std::chrono::milliseconds )
                    {
                        auto ptrUint8 = reinterpret_cast<uint8_t*>(ptr);
                        for(unsigned int i = 0; i < header.size(); ++i)
                        {
                            ptrUint8[i] = header[i];
                        }
                        return header.size();
                    });
    EXPECT_CALL(*dummyStream, read(_,
                                   sizeof(decltype(sizeOfFrame)), _)).InSequence(seq)
            .WillOnce(
                    [&sizeOfFrame] (void* ptr, unsigned int , std::chrono::milliseconds )
                    {
                        auto ptrUint8 = reinterpret_cast<uint8_t*>(ptr);
                        auto sizeOfFrameUint8 = reinterpret_cast<uint8_t*>(&sizeOfFrame);

                        for(unsigned int i = 0; i < sizeof(decltype(sizeOfFrame)); ++i)
                        {
                            ptrUint8[i] = sizeOfFrameUint8[i];
                        }
                        return sizeof(decltype(sizeOfFrame));
                    }
            );
    EXPECT_CALL(*dummyStream, read(_,
                                   dummyValueExpected.size()+1, _)).InSequence(seq)
            .WillOnce(Return(dummyValueExpected.size()+1));

    IoStream ioStream{std::move(dummyStream)};
    CygLidarD1 lidar{ioStream};
    lidar.readAndParse2dFrame();
    lidar.use2dPointCloud([](const CygLidarD1::PointCloud2D& ) {});
}

TEST(CygLidarD1Test, CygLidarD1DepthImageReadTestWrongHeaderCase)
{
    using namespace lidar_viewer::dev;
    using testing::_;
    using testing::Sequence;
    using testing::Return;
    Frame < CygLidarD1::FRAME_SIZE_3D > dummyValueExpected{};
    auto header = dummyValueExpected.header();

    auto dummyStream = std::make_unique<IoStreamMock>();
    Sequence seq;

    // just a single call at the end of scope,
    // don't care about the configuration, just
    // like with regular file reading
    EXPECT_CALL(*dummyStream, write(_, _, _));
    EXPECT_CALL(*dummyStream, close());

    EXPECT_CALL(*dummyStream, read(_,
                                   dummyValueExpected.header().size(), _)).InSequence(seq)
            .WillOnce(
                    [&header] (void* ptr, unsigned int , std::chrono::milliseconds )
                    {
                        auto ptrUint8 = reinterpret_cast<uint8_t*>(ptr);
                        for(unsigned int i = 0; i < header.size(); ++i)
                        {
                            ptrUint8[i] = 0;
                        }
                        return header.size();
                    });

    IoStream ioStream{std::move(dummyStream)};
    CygLidarD1 lidar{ioStream};
    lidar.readAndParse3dFrame();
}


TEST(CygLidarD1Test, CygLidarD1ConfigTest)
{
    using namespace lidar_viewer::dev;
    using testing::_;
    using testing::Sequence;
    using testing::Return;

    auto dummyStream = std::make_unique<IoStreamMock>();
    Sequence seq;

    EXPECT_CALL(*dummyStream, write(_, Req2{}.rawSize(), _)).InSequence(seq);
    EXPECT_CALL(*dummyStream, write(_, Req3{}.rawSize(), _)).InSequence(seq);
    EXPECT_CALL(*dummyStream, write(_, Req2{}.rawSize(), _)).InSequence(seq);
    EXPECT_CALL(*dummyStream, write(_, Req2{}.rawSize(), _)).InSequence(seq);
    EXPECT_CALL(*dummyStream, write(_, Req2{}.rawSize(), _)).InSequence(seq);
    EXPECT_CALL(*dummyStream, close());

    IoStream ioStream{std::move(dummyStream)};
    CygLidarD1 lidar{ioStream};
    CygLidarD1::Config lidarCfg{};
    lidar.configure(lidarCfg);
}

TEST(CygLidarD1Test, CygLidarD1PrintDeviceInfo)
{
    using namespace lidar_viewer::dev;
    using testing::_;
    using testing::Sequence;
    using testing::Return;

    Frame < 7u > dummyValueExpected{{16,0,3,5,0,2,2}};
    auto sizeOfFrame = dummyValueExpected.size();
    auto header = dummyValueExpected.header();

    auto dummyStream = std::make_unique<IoStreamMock>();
    Sequence seq;
    EXPECT_CALL(*dummyStream, write(_, Req2{}.rawSize(), _)).InSequence(seq);
    EXPECT_CALL(*dummyStream, close());

    EXPECT_CALL(*dummyStream, read(_,
                                   dummyValueExpected.header().size(), _)).InSequence(seq)
            .WillOnce(
                    [&header] (void* ptr, unsigned int , std::chrono::milliseconds )
                    {
                        auto ptrUint8 = reinterpret_cast<uint8_t*>(ptr);
                        for(unsigned int i = 0; i < header.size(); ++i)
                        {
                            ptrUint8[i] = header[i];
                        }
                        return header.size();
                    });
    EXPECT_CALL(*dummyStream, read(_,
                                   sizeof(decltype(sizeOfFrame)), _)).InSequence(seq)
            .WillOnce(
                    [&sizeOfFrame] (void* ptr, unsigned int , std::chrono::milliseconds )
                    {
                        auto ptrUint8 = reinterpret_cast<uint8_t*>(ptr);
                        auto sizeOfFrameUint8 = reinterpret_cast<uint8_t*>(&sizeOfFrame);

                        for(unsigned int i = 0; i < sizeof(decltype(sizeOfFrame)); ++i)
                        {
                            ptrUint8[i] = sizeOfFrameUint8[i];
                        }
                        return sizeof(decltype(sizeOfFrame));
                    }
            );
    EXPECT_CALL(*dummyStream, read(_,
                                   dummyValueExpected.size()+1, _)).InSequence(seq)
            .WillOnce(
                    [&dummyValueExpected] (void* ptr, unsigned int , std::chrono::milliseconds )
                    {
                        auto ptrUint8 = reinterpret_cast<uint8_t *>(ptr);
                        auto sizeOfFrameUint8 = dummyValueExpected.raw()
                                + dummyValueExpected.header().size() + 2;

                        for (unsigned int i = 0; i < dummyValueExpected.size(); ++i) {
                            ptrUint8[i] = sizeOfFrameUint8[i];
                        }
                        return dummyValueExpected.size() + 1;
                    }
                    );

    EXPECT_CALL(*dummyStream, write(_, Req2{}.rawSize(), _)).InSequence(seq);

    IoStream ioStream{std::move(dummyStream)};
    CygLidarD1 lidar{ioStream};
    testing::internal::CaptureStdout();
    lidar.printDeviceInfo();
    const auto output = testing::internal::GetCapturedStdout();
    ASSERT_EQ(output, "Device info: 16.0.3.5.0.2.2.\n");
}

} // namespace lidar_viewer::tests::units
