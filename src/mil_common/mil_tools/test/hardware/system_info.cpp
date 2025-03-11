#include "mil_tools/hardware/system_info.hpp"

#include <gtest/gtest.h>
#include <cstdlib>
#include <string>

TEST(system_info, get_cpu_architecture)
{
  // Create a pipe so I can execute "arch" and extract the architecture
  FILE* pipe = popen("arch", "r");
  
  // Buffer to contain the output
  char buffer[10];
  std::string operandA = "";
  
  // Continues until we reach the end of the pipe's output
  while (!feof(pipe)) 
  {
      if (fgets(buffer, 10, pipe) != nullptr) 
      {
          operandA += buffer;
      }
  }
  
  pclose(pipe);
  
  // Remove the newline (if any)
  if (!operandA.empty() && operandA[operandA.length()-1] == '\n') 
  {
      operandA.erase(operandA.length()-1);
  }

  // Store the result of the get_cpu_architecture function for our comparison
  std::string operandB = mil_tools::hardware::system_info::get_cpu_architecture();

  ASSERT_EQ(operandA.compare(operandB), 0);
}
