#include <cstdlib>
#include <gtest/gtest.h>

#include "mil_tools/fs/path.hpp"

TEST(mil_tools_fs, expanduser) {
  EXPECT_EQ(mil_tools::fs::expanduser("~"), std::getenv("HOME"));
  EXPECT_EQ(mil_tools::fs::expanduser("~/"), *mil_tools::fs::home() + "/");
  EXPECT_EQ(mil_tools::fs::expanduser("~/foo"),
            *mil_tools::fs::home() + "/foo");
  EXPECT_EQ(mil_tools::fs::expanduser("/foo"), "/foo");
  EXPECT_EQ(mil_tools::fs::expanduser(""), "");
}

TEST(mil_tools_fs, home) {
  const char *original_home = std::getenv("HOME");
  setenv("HOME", "/mock/home", 1);
  EXPECT_EQ(*mil_tools::fs::home(), "/mock/home");
  unsetenv("HOME");
  if (original_home)
    setenv("HOME", original_home, 1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
