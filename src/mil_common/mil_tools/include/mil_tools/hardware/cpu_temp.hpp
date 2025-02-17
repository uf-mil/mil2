#pragma once

#include <string>

namespace mil_tools::hardware::cpu_temp
{
    struct cpu_temperatures
    {
        double edge = -1;
        double temp1 = -1;
        double Tctl = -1;
        double Composite = -1;
        double Sensor1 = -1;
        double temp1 = -1;
    };


    /**
     * @brief Overloaded version of get_cpu_temperatures that takes a reference to a cpu_temperatures struct object to modify instead of returning a copy.
     * 
     * @param object 
     */
    void get_cpu_temperatures(cpu_temperatures& object);


    /**
     * @brief Returns a struct object containing CPU temperatures using libsensors.
     * 
     * @return cpu_temperatures 
     */
    cpu_temperatures get_cpu_temperatures();


    /**
     * @brief Retrieves a specific CPU temperature reading based on the argument string. Options: "edge", "temp1", "Tctl", "Composite", "Sensor 1", "temp1".
     * 
     * @param temperature_reading_name 
     * @return double 
     */
    double get_cpu_temperature(const std::string& temperature_reading_name);
}