#include "mil_tools/os.hpp"

#include "mil_tools/os/FileDescriptor.hpp"

namespace mil_tools::os
{
std::tuple<FileDescriptor, FileDescriptor> openpty()
{
    int master = -1, slave = -1;
    if (::openpty(&master, &slave, nullptr, nullptr, nullptr) == -1)
    {
        throw std::runtime_error("openpty failed: " + std::string(strerror(errno)));
    }
    if (grantpt(master) < 0)
    {
        throw std::runtime_error("grantpt failed: " + std::string(strerror(errno)));
    }
    if (unlockpt(master) < 0)
    {
        throw std::runtime_error("unlockpt failed: " + std::string(strerror(errno)));
    }
    return { FileDescriptor(master), FileDescriptor(slave) };
}

void close(int fd)
{
    if (::close(fd) == -1)
    {
        throw std::runtime_error("close failed: " + std::string(strerror(errno)));
    }
}

std::vector<char> read(int fd, size_t count)
{
    std::vector<char> data(count);
    ssize_t n = ::read(fd, data.data(), count);
    if (n == -1)
    {
        throw std::runtime_error("read failed: " + std::string(strerror(errno)));
    }
    data.resize(n);
    return data;
}

void write(int fd, std::vector<char> const& data)
{
    ssize_t total_written = 0;
    while (total_written < static_cast<ssize_t>(data.size()))
    {
        ssize_t n = ::write(fd, data.data() + total_written, data.size() - total_written);
        if (n == -1)
        {
            throw std::runtime_error("write failed: " + std::string(strerror(errno)));
        }
        total_written += n;
    }
}

void write(int fd, std::string const& data)
{
    write(fd, std::vector<char>(data.begin(), data.end()));
}

FileDescriptor open(std::string const& filename, int flags)
{
    auto fd = ::open(filename.c_str(), flags);
    if (fd < 0)
    {
        throw std::runtime_error("open failed: " + std::string(strerror(errno)));
    }
    return FileDescriptor(fd);
}
}  // namespace mil_tools::os
