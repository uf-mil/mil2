#pragma once

#include <string>

namespace mil_tools::hardware::cpu_temp
{
/**
 * @brief Returns a double with CPU temperature using libsensors.
 *
 * @return cpu_temperatures
 */
double get_cpu_temperature();
}  // namespace mil_tools::hardware::cpu_temp
