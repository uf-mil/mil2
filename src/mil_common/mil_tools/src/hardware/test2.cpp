#include <sensors-c++/sensors.h>
#include <iostream>

using namespace std;


int main()
{
sensors::subfeature temp {"/sys/devices/virtual/thermal/thermal_zone0/temp"};

cout << temp.read() << '\n';

return 0;
}
