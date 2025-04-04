#pragma once

#include <string.h>

#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace mil_tools::os
{
class FileDescriptor
{
  private:
    int fd_;

  public:
    FileDescriptor() : fd_(-1)
    {
    }
    explicit FileDescriptor(int fd) : fd_(fd)
    {
    }
    ~FileDescriptor();
    inline int get() const
    {
        return fd_;
    };
    inline bool valid() const
    {
        return fd_ >= 0;
    };
    explicit operator int() const;
    FileDescriptor(FileDescriptor const&) = delete;
    FileDescriptor& operator=(FileDescriptor const&) = delete;
    FileDescriptor(FileDescriptor&& other);
    FileDescriptor& operator=(FileDescriptor&& other);
    void close();
    std::vector<char> read(int size);
    std::string read_as_string(int size);
    void write(std::string const& data);
    void write(std::vector<char> const& data);
};

}  // namespace mil_tools::os
