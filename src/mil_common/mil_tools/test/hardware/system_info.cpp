#include "mil_tools/hardware/system_info.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

TEST(system_info, get_cpu_architecture)
{
    // Determines architecture
    FILE* pipe = popen("arch", "r");

    char buffer[10];
    std::string operandA = "";

    while (!feof(pipe))
    {
        if (fgets(buffer, 10, pipe) != nullptr)
        {
            operandA += buffer;
        }
    }

    pclose(pipe);

    // Remove the newline (if any)
    if (!operandA.empty() && operandA[operandA.length() - 1] == '\n')
    {
        operandA.erase(operandA.length() - 1);
    }

    // Store the result of the get_cpu_architecture function for our comparison
    std::string operandB = mil_tools::hardware::system_info::get_cpu_architecture();

    // compare() returns 0 if both strings are equal, so ASSERT_EQ's second argument can be 0 (value we're looking for)
    ASSERT_EQ(operandA.compare(operandB), 0);
}
