#include "lidar_viewer/geometry/types/ScreenRanges.h"

#include <gtest/gtest.h>

namespace lidar_viewer::tests::units
{

TEST(ScreenRangesTest, TestOpenGlScreenRanges)
{
    geometry::types::ScreenRangeGl openGlScreenRange{};

    const auto x = openGlScreenRange.fullRangeX();
    const auto y = openGlScreenRange.fullRangeY();
    const auto z = openGlScreenRange.fullRangeZ();

    ASSERT_EQ(x.first, -1.f);
    ASSERT_EQ(x.second, 1.f);

    ASSERT_EQ(y.first, 1.f);
    ASSERT_EQ(y.second, -1.f);

    ASSERT_EQ(z.first, 1.f);
    ASSERT_EQ(z.second, 0.f);
}

} // namespace lidar_viewer::tests::units
