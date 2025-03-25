#pragma once

#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "mil_tools/os/FileDescriptor.hpp"
#include "mil_tools/string.hpp"

namespace mil_tools::os
{

class TemporaryFile
{
  private:
    std::string name_;
    std::string tmp_path_ = std::filesystem::temp_directory_path().string();
    void create()
    {
        char* name = path().data();
        int raw_fd = mkstemp(name);
        if (raw_fd < 0)
        {
            throw std::runtime_error("Failed to create temporary file " + path() + ": " + strerror(errno));
        }
        name_ = mil_tools::string::removeprefix(name, tmp_path_);
        fd_ = FileDescriptor(raw_fd);
    }
    FileDescriptor fd_;

  public:
    TemporaryFile() : name_("tmpXXXXXX")
    {
        create();
    };
    explicit TemporaryFile(std::string const& name) : name_(name)
    {
        create();
    };
    inline std::string name() const
    {
        return name_;
    }
    inline std::string path() const
    {
        return tmp_path_ + "/" + name_;
    }
    ~TemporaryFile()
    {
        close();
    };
    inline bool valid()
    {
        return fd_.valid();
    }
    void close()
    {
        ::unlink(path().data());
        fd_.close();
    };
    void write(std::string const& data)
    {
        fd_.write(data);
        fsync(fd_.get());
    };
    void write(std::vector<char> const& data)
    {
        fd_.write(data);
        fsync(fd_.get());
    };
    std::string read_as_string(int size)
    {
        return fd_.read_as_string(size);
    };
    std::vector<char> read(int size)
    {
        return fd_.read(size);
    };
};

}  // namespace mil_tools::os
