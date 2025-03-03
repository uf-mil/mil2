#pragma once

#include <string>

namespace mil_tools::hardware::system_info
{
    /**
     * @brief Uses "uname -m" to get the CPU architecture.
     * 
     * @return std::string 
     */
    std::string get_cpu_architecture();
}