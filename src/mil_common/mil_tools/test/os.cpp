#include "mil_tools/os.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <iostream>

#include "mil_tools/fs/path.hpp"

TEST(mil_tools_os, open)
{
  auto fd_null = mil_tools::os::open("/dev/null", O_RDWR);
  EXPECT_TRUE(fd_null.valid());
  fd_null.close();
  EXPECT_FALSE(fd_null.valid());
  auto fd_readme = mil_tools::os::open(mil_tools::fs::path::expanduser("~/mil2/README.md"), O_RDONLY);
  EXPECT_EQ(fd_readme.read_as_string(5), "![Col");
}

TEST(mil_tools_os, write)
{
  // create file if not eist
  std::ofstream(mil_tools::fs::path::expanduser("~/mil2/tmp.txt"));

  // write with fd
  std::string filename = mil_tools::fs::path::expanduser("~/mil2/tmp.txt");
  auto fd = mil_tools::os::open(filename, O_RDWR | O_CREAT);
  mil_tools::os::write(fd.get(), "hello");
  fd.write(" world");
  fd.close();

  // check contents
  std::ifstream file(filename);
  std::string line;
  std::getline(file, line);
  EXPECT_EQ(line, "hello world");
  file.close();

  // delete file
  std::remove(filename.c_str());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
