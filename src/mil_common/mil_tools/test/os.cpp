#include "mil_tools/os.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <iostream>

#include "mil_tools/fs/path.hpp"
#include "mil_tools/os/TemporaryFile.hpp"

TEST(mil_tools_os, open)
{
  // TODO (cameron): fix this in CI :(
  // auto fd_readme = mil_tools::os::open(mil_tools::fs::path::expanduser("~/mil2/README.md"), O_RDONLY);
  // EXPECT_EQ(fd_readme.read_as_string(5), "![Col");
}

TEST(mil_tools_os, write)
{
  // create file if not eist
  mil_tools::os::TemporaryFile tmpfile;
  tmpfile.write("hello world");

  // check contents
  std::ifstream file(tmpfile.path());
  assert(file.is_open());
  std::string line;
  std::getline(file, line);
  EXPECT_EQ(line, "hello world");
  file.close();
  tmpfile.close();

  // ensure temp file was actually deleted
  file.open(tmpfile.path());
  EXPECT_FALSE(file.is_open());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
