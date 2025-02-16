#include "mil_tools/itertools.hpp"

#include <gtest/gtest.h>

#include <cstdlib>

TEST(mil_tools_itertools, enumerate)
{
  std::vector<std::pair<int, int>> nums = {{0, 10}, {1, 20}, {2, 30}};
  std::vector<std::pair<int, std::string>> strings = {{0, "first"}, {1, "second"}, {2, "third"}};
  EXPECT_EQ(mil_tools::itertools::enumerate({10, 20, 30}), nums);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}