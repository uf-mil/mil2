#include "mil_tools/hardware/system_info.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

namespace mil_tools::hardware::system_info
{
    std::string get_cpu_architecture() 
    {
        // Execute "uname -m"
        FILE* pipe = popen("uname -m", "r");

        // Null pointer means we failed to execute the command so return "Unknown"
        if (!pipe) 
        {
            return "Unknown";
        }
        
        char buffer[128];
        std::string result = "";
        
        while (!feof(pipe)) 
        {
            if (fgets(buffer, 128, pipe) != nullptr) 
            {
                result += buffer;
            }
        }
        
        pclose(pipe);
        
        // Remove the newline, then we can return the result
        if (!result.empty() && result[result.length()-1] == '\n') 
        {
            result.erase(result.length()-1);
        }
        
        return result;
    }
}