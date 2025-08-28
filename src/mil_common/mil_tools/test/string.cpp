#include "mil_tools/string.hpp"

#include <gtest/gtest.h>

#include <cstdlib>

TEST(mil_tools_string, split)
{
    std::vector<std::string> all_three{ "a", "b", "c" };
    std::vector<std::string> all_one{ "a,b,c" };
    EXPECT_EQ(mil::string::split("a,b,c", ","), all_three);
    EXPECT_EQ(mil::string::split("a b c", " "), all_three);
    EXPECT_EQ(mil::string::split("a,b,c", " "), all_one);
    EXPECT_EQ(mil::string::split("", ","), std::vector<std::string>{ "" });
}

TEST(mil_tools_string, startswith)
{
    EXPECT_EQ(mil::string::startswith("abc", "a"), true);
    EXPECT_EQ(mil::string::startswith("abc", "ab"), true);
    EXPECT_EQ(mil::string::startswith("abc", "abc"), true);
    EXPECT_EQ(mil::string::startswith("abc", "abcd"), false);
    EXPECT_EQ(mil::string::startswith("abc", "b"), false);
    EXPECT_EQ(mil::string::startswith("abc", ""), true);
    EXPECT_EQ(mil::string::startswith("", ""), true);
}

TEST(mil_tools_string, endswith)
{
    EXPECT_EQ(mil::string::endswith("abc", "c"), true);
    EXPECT_EQ(mil::string::endswith("abc", "bc"), true);
    EXPECT_EQ(mil::string::endswith("abc", "abc"), true);
    EXPECT_EQ(mil::string::endswith("abc", "zabc"), false);
    EXPECT_EQ(mil::string::endswith("abc", "b"), false);
    EXPECT_EQ(mil::string::endswith("abc", ""), true);
    EXPECT_EQ(mil::string::endswith("", ""), true);
}

TEST(mil_tools_string, lstrip)
{
    EXPECT_EQ(mil::string::lstrip("  abc"), "abc");
    EXPECT_EQ(mil::string::lstrip("abc"), "abc");
    EXPECT_EQ(mil::string::lstrip("abc  "), "abc  ");
    EXPECT_EQ(mil::string::lstrip("  "), "");
    EXPECT_EQ(mil::string::lstrip(""), "");
}

TEST(mil_tools_string, rstrip)
{
    EXPECT_EQ(mil::string::rstrip("abc  "), "abc");
    EXPECT_EQ(mil::string::rstrip("abc"), "abc");
    EXPECT_EQ(mil::string::rstrip("  abc"), "  abc");
    EXPECT_EQ(mil::string::rstrip("  "), "");
    EXPECT_EQ(mil::string::rstrip(""), "");
}

TEST(mil_tools_string, strip)
{
    EXPECT_EQ(mil::string::strip("  abc  "), "abc");
    EXPECT_EQ(mil::string::strip("abc"), "abc");
    EXPECT_EQ(mil::string::strip("  abc"), "abc");
    EXPECT_EQ(mil::string::strip("abc  "), "abc");
    EXPECT_EQ(mil::string::strip("  "), "");
    EXPECT_EQ(mil::string::strip(""), "");
}

TEST(mil_tools_string, removeprefix)
{
    EXPECT_EQ(mil::string::removeprefix("abc", "a"), "bc");
    EXPECT_EQ(mil::string::removeprefix("abc", "ab"), "c");
    EXPECT_EQ(mil::string::removeprefix("abc", "abc"), "");
    EXPECT_EQ(mil::string::removeprefix("abc", "zabc"), "abc");
    EXPECT_EQ(mil::string::removeprefix("abc", "b"), "abc");
    EXPECT_EQ(mil::string::removeprefix("abc", ""), "abc");
    EXPECT_EQ(mil::string::removeprefix("", ""), "");
}

TEST(mil_tools_string, removesuffix)
{
    EXPECT_EQ(mil::string::removesuffix("abc", "c"), "ab");
    EXPECT_EQ(mil::string::removesuffix("abc", "bc"), "a");
    EXPECT_EQ(mil::string::removesuffix("abc", "abc"), "");
    EXPECT_EQ(mil::string::removesuffix("abc", "zabc"), "abc");
    EXPECT_EQ(mil::string::removesuffix("abc", "b"), "abc");
    EXPECT_EQ(mil::string::removesuffix("abc", ""), "abc");
    EXPECT_EQ(mil::string::removesuffix("", ""), "");
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
