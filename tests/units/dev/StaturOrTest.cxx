#include "lidar_viewer/dev/StatusOr.h"
#include "lidar_viewer/dev/BinaryFile.h"
#include <gtest/gtest.h>

namespace lidar_viewer::tests::units
{

TEST(StatusOrTest, MakeStatusOr)
{
    using namespace lidar_viewer::dev;
    StatusOr<int> MyIntStatus{MakeStatusOr<int>(10)};
    EXPECT_EQ(MyIntStatus.value(), 10);
}

} // namespace lidar_viewer::tests::units
