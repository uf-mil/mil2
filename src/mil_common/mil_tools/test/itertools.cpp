#include "mil_tools/itertools.hpp"

#include <gtest/gtest.h>

#include <cstdlib>

TEST(mil_tools_itertools, enumerate)
{
  std::vector<std::pair<int, int>> exp_nums = {{0, 10}, {1, 20}, {2, 30}};
  std::vector<int> sample_nums = {10, 20, 30};
  std::vector<std::pair<int, int>> test_nums;
  for (const auto& [index, value] : mil_tools::itertools::enumerate(sample_nums))
  {
    test_nums.push_back({index, value});
  }
  EXPECT_EQ(test_nums, exp_nums);

  std::vector<std::pair<int, std::string>> exp_strings = {{0, "first"}, {1, "second"}, {2, "third"}};
  std::vector<std::string> sample_strings = {"first", "second", "third"};
  std::vector<std::pair<int, std::string>> test_strings;
  for (const auto& [index, value] : mil_tools::itertools::enumerate(sample_strings))
  {
    test_strings.push_back({index, value});
  }
  EXPECT_EQ(test_strings, exp_strings);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}