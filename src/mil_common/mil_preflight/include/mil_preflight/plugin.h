#pragma once

#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

namespace mil_preflight
{
    class PluginBase: public rclcpp::Node
    {
        public:
        PluginBase(std::string const& name):
            rclcpp::Node(name),
            workThread_(boost::bind(&PluginBase::runTest, this))
        {

        }

        ~PluginBase()
        {
            workThread_.join();
        }

        virtual bool runAction(std::string const& parameter)
        {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
            RCLCPP_ERROR(get_logger(), "No such plugin: %s", get_name());
            return false;
        }

        private:

        boost::thread workThread_;

        void runTest()
        {
            std::string line;
            while(std::getline(std::cin, line))
            {
                if(line[0] == char(0x04))
                    break;
                
                bool success = runAction(line);
                std::cout << (success ? char(0x06) : char(0x15))  << std::endl;
                std::cerr << char(0x04) << std::endl;
            }

            rclcpp::shutdown();
        }
    };
}