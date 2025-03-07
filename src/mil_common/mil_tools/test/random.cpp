#include "mil_tools/random.hpp"

#include <gtest/gtest.h>

TEST(mil_tools_random, random_string)
{
    std::string str = mil_tools::random::random_string(10);
    EXPECT_EQ(str.size(), 10);
    std::string str2 = mil_tools::random::random_string(10);
    EXPECT_EQ(str2.size(), 10);
    EXPECT_NE(str, str2);
}
