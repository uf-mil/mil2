#include <sensors-c++/sensors.h>
#include <iostream>
#include "/home/praveen/mil2/src/mil_common/mil_tools/include/mil_tools/hardware/system_info.hpp"
#include "/home/praveen/mil2/src/mil_common/mil_tools/include/mil_tools/hardware/cpu_temp.hpp"

using namespace std;


int main()
{
    cout << mil_tools::hardware::cpu_temp::get_cpu_temperature();
}
