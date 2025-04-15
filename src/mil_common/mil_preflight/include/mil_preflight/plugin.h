#pragma once

#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/chrono.hpp>
#include <boost/dll.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mil_preflight/common.h"

namespace mil_preflight
{
class PluginBase : public rclcpp::Node
{
  public:
    PluginBase() : rclcpp::Node("mil_preflight_node"), workThread_(boost::bind(&PluginBase::runTest, this))
    {
    }

    virtual ~PluginBase()
    {
        workThread_.join();
    }

    static std::shared_ptr<PluginBase> create(std::string const& pluginName)
    {
        try
        {
            creator_ = boost::dll::import_alias<Creator>(pluginName, pluginName,
                                                         boost::dll::load_mode::append_decorations |
                                                         boost::dll::load_mode::search_system_folders);
        }
        catch (boost::system::system_error const& e)
        {
            error_ = "Failed to load plugin: " + pluginName;
            std::shared_ptr<PluginBase> plugin = std::make_shared<PluginBase>();
            RCLCPP_ERROR(plugin->get_logger(), e.what());
            return plugin;
        }

        return creator_();
    }

  protected:
    virtual bool runAction([[maybe_unused]] std::vector<std::string>&& parameters)
    {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
        return false;
    }

    virtual std::string const& getSummery()
    {
        return error_;
    }

    int askQuestion(std::string const& question, std::vector<std::string> const& options)
    {
        std::ostringstream oss;
        oss << BEL << std::endl << question << GS;
        for (std::string const& option : options)
        {
            oss << option << GS;
        }

        oss << EOT << GS;

        std::cout << std::move(oss.str());

        std::string line;
        if (!std::getline(std::cin, line))
            return -1;

        int index = -1;
        try
        {
            index = std::stoi(line);
        }
        catch (std::exception const& e)
        {
        }

        return index;
    }

  private:
    using Creator = std::shared_ptr<PluginBase>();

    boost::thread workThread_;
    static std::string error_;
    static boost::function<Creator> creator_;

    void runTest()
    {
        std::string line;
        std::vector<std::string> parameters;
        while (std::getline(std::cin, line))
        {
            if (line[0] == EOT)
            {
                break;
            }
            else if (line[0] == GS)
            {
                bool success = runAction(std::move(parameters));
                std::ostringstream stdoutss;
                stdoutss << (success ? ACK : NCK) << std::endl;
                stdoutss << getSummery() << std::endl;
                std::cout << std::move(stdoutss.str());

                std::ostringstream stderrss;
                stderrss << EOT << std::endl;
                std::cerr << std::move(stderrss.str());

                parameters.clear();
            }
            else
            {
                parameters.push_back(std::move(line));
            }
        }

        rclcpp::shutdown();
    }
};

std::string PluginBase::error_ = "success";
boost::function<PluginBase::Creator> PluginBase::creator_;
}  // namespace mil_preflight
