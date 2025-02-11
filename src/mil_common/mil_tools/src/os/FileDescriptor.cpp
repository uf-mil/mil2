#include "mil_tools/os/FileDescriptor.hpp"

#include "mil_tools/os.hpp"

namespace mil_tools::os
{
FileDescriptor::~FileDescriptor()
{
  close();
}

FileDescriptor::operator int() const
{
  return fd_;
};

FileDescriptor::FileDescriptor(FileDescriptor&& other)
{
  fd_ = other.fd_;
  other.fd_ = -1;
}

FileDescriptor& FileDescriptor::operator=(FileDescriptor&& other)
{
  if (this != &other)
  {
    fd_ = other.fd_;
    other.fd_ = -1;
  }
  return *this;
}

void FileDescriptor::close()
{
  if (fd_ != -1)
  {
    mil_tools::os::close(fd_);
    fd_ = -1;
  }
}

std::vector<char> FileDescriptor::read(int size)
{
  return mil_tools::os::read(fd_, size);
}

std::string FileDescriptor::read_as_string(int size)
{
  std::vector<char> raw = read(size);
  return std::string(raw.begin(), raw.end());
}

void FileDescriptor::write(std::vector<char> const& data)
{
  mil_tools::os::write(fd_, data);
}

void FileDescriptor::write(std::string const& data)
{
  mil_tools::os::write(fd_, data);
}

}  // namespace mil_tools::os
