#pragma once

#include <boost/json.hpp>
#include <boost/filesystem.hpp>

#include <fstream>
#include <filesystem>

namespace mil_preflight
{
    class Configuration
    {
        public:
        Configuration()
        {
            auto binPath = std::filesystem::canonical("/proc/self/exe").parent_path();
            this->Configuration(binPath / ".." / "cfg" / "config.json");
        }

        Configuration(std::string const& fileName)
        {
            
        }

        ~Configuration()
        {
            
        }

        private:

        void parse_(std::string const& fileName)
        {
            std::ifstream file(fileName);
            if(!file.is_open())
                throw std::runtime_error("Could not find configuration file: " + fileName);

            boost::json::value data_ = std::move(boost::json::parse(file));
            file.close();
        }

    };
}