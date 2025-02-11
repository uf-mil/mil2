/*
 * os.hpp - High-level OS utilities
 */
#pragma once

#include <fcntl.h>
#include <pty.h>
#include <string.h>
#include <unistd.h>

#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "mil_tools/os/FileDescriptor.hpp"

namespace mil_tools::os
{
/// Opens a file
FileDescriptor open(std::string const& filename, int flags);
/// Open a new pseudo-terminal pair (master, slave)
std::tuple<FileDescriptor, FileDescriptor> openpty();
/// Closes a file descriptor
void close(int fd);
/// Reads a certain number of bytes from a file descriptor
std::vector<char> read(int fd, size_t count);
/// Writes a string to a file descriptor
void write(int fd, std::vector<char> const& data);
void write(int fd, std::string const& data);
};  // namespace mil_tools::os
