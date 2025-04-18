#include "mil_tools/math.hpp"

#include <gtest/gtest.h>

TEST(mil_tools_math, is_close_exact_equal)
{
    EXPECT_TRUE(mil_tools::math::is_close(0.1f, 0.1f));
    EXPECT_TRUE(mil_tools::math::is_close(1.0, 1.0));
}

TEST(mil_tools_math, is_close_within_default_epsilon)
{
    float a = 1.0000001f;
    float b = 1.0000002f;
    EXPECT_TRUE(mil_tools::math::is_close(a, b));
}

TEST(mil_tools_math, is_not_close_beyond_default_epsilon)
{
    float a = 1.0f;
    float b = 1.0001f;
    EXPECT_FALSE(mil_tools::math::is_close(a, b));
}

TEST(mil_tools_math, is_close_custom_epsilon)
{
    double a = 100.0;
    double b = 100.5;
    EXPECT_TRUE(mil_tools::math::is_close(a, b, 0.01));       // false
    EXPECT_TRUE(mil_tools::math::is_close(a, b, 0.01 * 10));  // true
}

TEST(mil_tools_math, is_close_with_zero)
{
    EXPECT_TRUE(mil_tools::math::is_close(0.0f, 0.0f));
    EXPECT_FALSE(mil_tools::math::is_close(0.0f, 1e-5f));        // too large relative to 0
    EXPECT_TRUE(mil_tools::math::is_close(0.0f, 1e-9f, 1e-6f));  // close enough with custom epsilon
}

TEST(mil_tools_math, is_close_negative_numbers)
{
    EXPECT_TRUE(mil_tools::math::is_close(-1.0f, -1.0f));
    EXPECT_TRUE(mil_tools::math::is_close(-1.0f, -1.0000001f));
    EXPECT_FALSE(mil_tools::math::is_close(-1.0f, -1.1f));
}

TEST(mil_tools_math, is_close_large_values)
{
    double a = 1e10;
    double b = 1e10 + 1e4;
    EXPECT_TRUE(mil_tools::math::is_close(a, b, 1e-5));
    EXPECT_FALSE(mil_tools::math::is_close(a, b));  // default epsilon too small
}
