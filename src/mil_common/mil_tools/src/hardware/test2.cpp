#include <sensors-c++/sensors.h>
#include <iostream>

using namespace std;


int main()
{
sensors::subfeature temp {"/sys/class/thermal/hwmon0/temp1_input"};

cout << temp.read() << '\n';

return 0;
}