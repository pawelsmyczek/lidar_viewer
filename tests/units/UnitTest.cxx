#include "StatusOr.h"
#include "lidar_viewer/dev/BinaryFile.h"
#include <gtest/gtest.h>

namespace lidar_viewer::tests::units
{

using namespace std::string_literals;

TEST(StatusOrTest, MakeStatusOr) {
    StatusOr::StatusOr<int> MyIntStatus{StatusOr::MakeStatusOr<int>(10)};
    EXPECT_EQ(MyIntStatus.value(), 10);
}

TEST(BinaryFileTest, TestBinaryFileOpenOkCase)
{
    const auto dummyBinaryFile = "dummy.bin"s;
    ::system(("touch " + dummyBinaryFile).c_str());
    ASSERT_NO_THROW(lidar_viewer::dev::BinaryFile{"dummy.bin"});
    ::system(("rm " + dummyBinaryFile).c_str());
}

TEST(BinaryFileTest, TestBinaryFileOpenFailCase)
{
    ASSERT_THROW(lidar_viewer::dev::BinaryFile{"NonExistingFile"},
                 std::runtime_error);
}

TEST(BinaryFileTest, TestBinaryFileReadWriteOkCase)
{
    const auto dummyBinaryFile = "dummy.bin"s;
    constexpr auto expectedValue = 2137;
    ::system(("touch " + dummyBinaryFile).c_str());
    // write
    {
        lidar_viewer::dev::BinaryFile binaryFile{"dummy.bin"};
        binaryFile.write(&expectedValue, sizeof(expectedValue), false);
    }

    // read back
    {
        std::remove_const_t<decltype(expectedValue)> returnedValue{};
        lidar_viewer::dev::BinaryFile binaryFile{"dummy.bin"};
        ASSERT_EQ(sizeof(returnedValue), binaryFile.read(&returnedValue, sizeof(returnedValue), {}));
        ASSERT_EQ(expectedValue, returnedValue);
    }
    ::system(("rm " + dummyBinaryFile).c_str());
}

TEST(BinaryFileTest, TestBinaryFileReadWriteDiscardOutputTrue)
{
    const auto dummyBinaryFile = "dummy.bin"s;
    constexpr auto expectedValue = 0;
    ::system(("touch " + dummyBinaryFile).c_str());
    // write
    {
        lidar_viewer::dev::BinaryFile binaryFile{"dummy.bin"};
        binaryFile.write(&expectedValue, sizeof(expectedValue), true);
    }

    // read back, don't check size returned
    {
        std::remove_const_t<decltype(expectedValue)> returnedValue{};
        lidar_viewer::dev::BinaryFile binaryFile{"dummy.bin"};
        binaryFile.read(&returnedValue, sizeof(returnedValue), {});
        ASSERT_EQ(expectedValue, returnedValue);
    }
    ::system(("rm " + dummyBinaryFile).c_str());
}

TEST(BinaryFileTest, TestBinaryFileReadEof)
{
    const auto dummyBinaryFile = "dummy.bin"s;
    std::array<uint8_t, 2> dummyArray{};
    ::system(("touch " + dummyBinaryFile).c_str());
    lidar_viewer::dev::BinaryFile binaryFile{"dummy.bin"};
    // first pass
    binaryFile.read(&dummyArray, sizeof(dummyArray), {});
    // shall throw
    ASSERT_THROW(binaryFile.read(&dummyArray, sizeof(dummyArray), {}), std::runtime_error);
    ::system(("rm " + dummyBinaryFile).c_str());
}

} // namespace lidar_viewer::tests::units

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
