#include "mil_tools/fs/path.hpp"

#include <gtest/gtest.h>

#include <cstdlib>

TEST(mil_tools_fs_path, expanduser)
{
    EXPECT_EQ(mil::fs::path::expanduser("~"), std::getenv("HOME"));
    EXPECT_EQ(mil::fs::path::expanduser("~/"), *mil::fs::path::home() + "/");
    EXPECT_EQ(mil::fs::path::expanduser("~/foo"), *mil::fs::path::home() + "/foo");
    EXPECT_EQ(mil::fs::path::expanduser("/foo"), "/foo");
    EXPECT_EQ(mil::fs::path::expanduser(""), "");
}

TEST(mil_tools_fs_path, home)
{
    char const *original_home = std::getenv("HOME");
    setenv("HOME", "/mock/home", 1);
    EXPECT_EQ(*mil::fs::path::home(), "/mock/home");
    unsetenv("HOME");
    if (original_home)
        setenv("HOME", original_home, 1);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
