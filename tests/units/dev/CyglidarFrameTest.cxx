#include "lidar_viewer/dev/CygLidarFrame.h"
#include "lidar_viewer/dev/BinaryFile.h"

#include <array>
#include <gtest/gtest.h>


namespace lidar_viewer::tests::units
{

static constexpr std::array<uint8_t, 4u + 6u> expected{0x5au, 0x77u, 0xffu, 0x04u, 0x00u, 0x02, 0x01, 0x03, 0x07, 0x04};

using namespace std::string_literals;

TEST(CygLidarFrameTest, CygLidarCorrectFrameTest)
{
    using namespace lidar_viewer::dev;
    Frame<4u> testCorrectFrame{{0x02, 0x01, 0x03, 0x07} };
    // test payload()
    {
        const auto& pload = *testCorrectFrame.payload();
        for(auto i = 0u; i < testCorrectFrame.size(); ++i)
        {
            ASSERT_EQ(pload[i], expected[i+5]);
        }
    }
    // test status()
    {
        ASSERT_EQ(testCorrectFrame.status(), Status::OK);
    }
    // test raw()
    {
        auto pload = testCorrectFrame.raw();
        for(auto i = 0u; i < testCorrectFrame.rawSize(); ++i)
        {
            ASSERT_EQ(pload[i], expected[i]);
        }
    }
    // test checksum()
    {
        ASSERT_EQ(testCorrectFrame.checksum(), expected[expected.size()-1]);
    }
}

TEST(CygLidarFrameTest, CygLidarIncorrectFrameTest)
{
    using namespace lidar_viewer::dev;
    Frame<4u> testIncorrectFrame{ };
    // test checksum
    {
        ASSERT_FALSE(testIncorrectFrame.validateChecksum(expected[expected.size()-1]));
    }
}

TEST(CygLidarFrameTest, CygLidarFrameReadWriteTest)
{
    const auto dummyBinaryFile = "dummy.bin"s;
    using namespace lidar_viewer::dev;
    auto payload = Frame<4u>::Payload {0x02, 0x01, 0x03, 0x07};
    Frame<4u> testFrame{payload};

    ::system(("touch " + dummyBinaryFile).c_str());

    // write
    {
        IoStream input{};

        input.createAndOpen<BinaryFile>("dummy.bin");

        write<4u>(testFrame, input);
    }

    // read tests
    {
        IoStream output{};
        output.createAndOpen<BinaryFile>("dummy.bin");
        auto returnedFrame = read<Frame<4u>>(output);

        ASSERT_TRUE(returnedFrame.ok());

        const auto & retPayload = *returnedFrame.value().payload();

        for(auto i = 0u; i < retPayload.size(); ++i)
        {
            ASSERT_EQ(retPayload[i], payload[i]);
        }
        ASSERT_TRUE(returnedFrame.value().validateChecksum(testFrame.checksum()));
    }

    {
        IoStream output{};
        output.createAndOpen<BinaryFile>("dummy.bin");
        Frame<4u> returnedFrame;
        auto returnedFrameStatus = read(output, returnedFrame);

        ASSERT_TRUE(returnedFrameStatus == Status::OK);

        const auto & retPayload = *returnedFrame.payload();

        for(auto i = 0u; i < retPayload.size(); ++i)
        {
            ASSERT_EQ(retPayload[i], payload[i]);
        }
        ASSERT_TRUE(returnedFrame.validateChecksum(testFrame.checksum()));
    }

    ::system(("rm " + dummyBinaryFile).c_str());
}

} // namespace lidar_viewer::tests::units
