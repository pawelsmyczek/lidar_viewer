#include "StatusOr.h"
#include <gtest/gtest.h>

TEST(StatusOrTest, MakeStatusOr) {
  StatusOr::StatusOr<int> MyIntStatus{StatusOr::MakeStatusOr<int>(10)};
  EXPECT_EQ(MyIntStatus.value(), 10);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
