#include <sensors-c++/sensors.h>
#include <iostream>
#include "mil_tools/hardware/system_info.hpp"
#include "mil_tools/hardware/cpu_temp.hpp"

using namespace std;


// Compilation Command: g++ -I../../include/ test2.cpp cpu_temp.cpp system_info.cpp -lsensors

int main()
{
    cout << mil_tools::hardware::cpu_temp::get_cpu_temperature();
}
