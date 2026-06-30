#include "log_to_file.hpp"

#include <filesystem>
#include <fstream>
#include <string>

BT::PortsList LogToFile::providedPorts()
{
    return {
        BT::InputPort<std::string>("dir", "/tmp", "Directory to write the file into"),
        BT::InputPort<std::string>("filename", "bt_log.txt", "Name of the file"),
        BT::InputPort<std::string>("message", "", "Text to append"),
    };
}

BT::NodeStatus LogToFile::tick()
{
    std::string dir, filename, message;
    if (!getInput("dir", dir) || !getInput("filename", filename) || !getInput("message", message))
        return BT::NodeStatus::FAILURE;

    std::filesystem::path path = std::filesystem::path(dir) / filename;
    std::ofstream f(path, std::ios::app);
    if (!f.is_open())
        return BT::NodeStatus::FAILURE;

    f << message << "\n";
    return BT::NodeStatus::SUCCESS;
}
