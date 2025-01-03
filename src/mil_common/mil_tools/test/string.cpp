#include "mil_tools/string.hpp"

#include <gtest/gtest.h>

#include <cstdlib>

TEST(mil_tools_string, split)
{
  std::vector<std::string> all_three{ "a", "b", "c" };
  std::vector<std::string> all_one{ "a,b,c" };
  EXPECT_EQ(mil_tools::string::split("a,b,c", ","), all_three);
  EXPECT_EQ(mil_tools::string::split("a b c", " "), all_three);
  EXPECT_EQ(mil_tools::string::split("a,b,c", " "), all_one);
  EXPECT_EQ(mil_tools::string::split("", ","), std::vector<std::string>{ "" });
}

TEST(mil_tools_string, startswith)
{
  EXPECT_EQ(mil_tools::string::startswith("abc", "a"), true);
  EXPECT_EQ(mil_tools::string::startswith("abc", "ab"), true);
  EXPECT_EQ(mil_tools::string::startswith("abc", "abc"), true);
  EXPECT_EQ(mil_tools::string::startswith("abc", "abcd"), false);
  EXPECT_EQ(mil_tools::string::startswith("abc", "b"), false);
  EXPECT_EQ(mil_tools::string::startswith("abc", ""), true);
  EXPECT_EQ(mil_tools::string::startswith("", ""), true);
}

TEST(mil_tools_string, endswith)
{
  EXPECT_EQ(mil_tools::string::endswith("abc", "c"), true);
  EXPECT_EQ(mil_tools::string::endswith("abc", "bc"), true);
  EXPECT_EQ(mil_tools::string::endswith("abc", "abc"), true);
  EXPECT_EQ(mil_tools::string::endswith("abc", "zabc"), false);
  EXPECT_EQ(mil_tools::string::endswith("abc", "b"), false);
  EXPECT_EQ(mil_tools::string::endswith("abc", ""), true);
  EXPECT_EQ(mil_tools::string::endswith("", ""), true);
}

TEST(mil_tools_string, lstrip)
{
  EXPECT_EQ(mil_tools::string::lstrip("  abc"), "abc");
  EXPECT_EQ(mil_tools::string::lstrip("abc"), "abc");
  EXPECT_EQ(mil_tools::string::lstrip("abc  "), "abc  ");
  EXPECT_EQ(mil_tools::string::lstrip("  "), "");
  EXPECT_EQ(mil_tools::string::lstrip(""), "");
}

TEST(mil_tools_string, rstrip)
{
  EXPECT_EQ(mil_tools::string::rstrip("abc  "), "abc");
  EXPECT_EQ(mil_tools::string::rstrip("abc"), "abc");
  EXPECT_EQ(mil_tools::string::rstrip("  abc"), "  abc");
  EXPECT_EQ(mil_tools::string::rstrip("  "), "");
  EXPECT_EQ(mil_tools::string::rstrip(""), "");
}

TEST(mil_tools_string, strip)
{
  EXPECT_EQ(mil_tools::string::strip("  abc  "), "abc");
  EXPECT_EQ(mil_tools::string::strip("abc"), "abc");
  EXPECT_EQ(mil_tools::string::strip("  abc"), "abc");
  EXPECT_EQ(mil_tools::string::strip("abc  "), "abc");
  EXPECT_EQ(mil_tools::string::strip("  "), "");
  EXPECT_EQ(mil_tools::string::strip(""), "");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
