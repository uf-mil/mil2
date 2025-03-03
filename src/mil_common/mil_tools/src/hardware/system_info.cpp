#include "hardware/system_info.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

std::string get_cpu_architecture() 
{
    // Create a pipe so I can execute "u-name -m" and extract the architecture
    FILE* pipe = popen("uname -m", "r");

    // If the pointer's null, that means we failed to execute the command so I'll return Unknown
    if (!pipe) 
    {
        return "Unknown";
    }
    
    // Buffer to contain the output
    char buffer[128];
    std::string result = "";
    
    // Continues until we reach the end of the file
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
