#pragma once

#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mil_preflight/common.h"

namespace mil_preflight
{
class PluginBase : public rclcpp::Node
{
  public:
    PluginBase() : rclcpp::Node("mil_preflight_node")
    {
    }

    virtual ~PluginBase()
    {
    }

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
                auto [success, summery] = runAction(std::move(parameters));
                std::ostringstream stdoutss;
                stdoutss << (success ? ACK : NCK) << std::endl;
                stdoutss << std::move(summery) << std::endl;
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
    }

  protected:
    virtual std::pair<bool, std::string> runAction([[maybe_unused]] std::vector<std::string>&& parameters)
    {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
        return { false, "Failed to load plugin" };
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
};

}  // namespace mil_preflight
