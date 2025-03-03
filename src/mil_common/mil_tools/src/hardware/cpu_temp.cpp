#include "mil_tools/hardware/cpu_temp.hpp"
#include "mil_tools/hardware/system_info.hpp"

#include <sensors/sensors.h>

namespace mil_tools::hardware::cpu_temp
{
    void get_cpu_temperatures(cpu_temperatures& object)
    {
        // First we need to know the CPU architecture since the command/library we'll use for determining the temp differs based on the architecture
        std::string cpu_architecture = mil_tools::hardware::system_info::get_cpu_architecture();

        // If the architecture is x86, we'll use the lm-sensors/libsensors-cpp library
        if (cpu_architecture.compare("x86_64") == 0)
        {
            // Initialize sensors, none-zero returned value means there was an error during initialization
            if (sensors_init(nullptr) != 0) 
            {
                // Return without modifying any values
                return;
            }

            // Iterate through all chips
            const sensors_chip_name *chip;
            
            // Specify starting chip as 0 so that we go through all available chips
            int chip_nr = 0;
            
            // Pass a nullptr for the sensors_chip_name argument to tell the function to read all available chips
            // since we might change our CPU or components.
            while ((chip = sensors_get_detected_chips(nullptr, &chip_nr)) != nullptr) 
            {
                // Get all features (sensors) of the chip
                const sensors_feature *feature;
                int feature_nr = 0;
                
                while ((feature = sensors_get_features(chip, &feature_nr))) 
                {
                    // We only care about temperature readings so any feature that's not temp we can skip
                    if (feature->type != SENSORS_FEATURE_TEMP) 
                    {
                        continue;
                    }

                    // Get temperature reading
                    double temp;
                    const sensors_subfeature *subfeature = sensors_get_subfeature(chip, feature, SENSORS_SUBFEATURE_TEMP_INPUT);
                        
                    // Make sure we didn't get a nullptr
                    if (subfeature) 
                    {
                        if (sensors_get_value(chip, subfeature->number, &temp) == 0) 
                        {
                            // Get sensor label
                            char *label = sensors_get_label(chip, feature);

                            if (label == "edge")
                                object.edge = temp;
                            else if (label == "temp1")
                                object.temp1 = temp;
                            else if (label == "Tctl")
                                object.Tctl = temp;
                            else if (label == "Composite")
                                object.Composite = temp;
                            else if (label == "Sensor 1")
                                object.Sensor1 = temp;
                            else if (label == "temp1")
                                object.temp1 = temp;

                            // Free the string that I used to store the temperature reading's label/identifier
                            free(label);
                        }
                    }
                }
            }

            sensors_cleanup();
        }

        // If the architecture is ARM, we can assume the device is a Jetson and use tegrastats to grab CPU temp
        else if (cpu_architecture.find("aarch") != std::string::npos || cpu_architecture.find("arm") != std::string::npos)
        {
            
        }

        // If the the get_cpu_architecture() function returns unknown, then we'll just return without modifying the cpu_temperatures object
        else
            return;
    }


    /**
     * @brief Returns a struct object containing CPU temperatures using libsensors.
     * 
     * @return cpu_temperatures 
     */
    cpu_temperatures get_cpu_temperatures()
    {
        cpu_temperatures cpu_temps_object;

        get_cpu_temperatures(cpu_temps_object);

        return cpu_temps_object;
    }


    /**
     * @brief Retrieves a specific CPU temperature reading based on the argument string. Options: "edge", "temp1", "Tctl", "Composite", "Sensor 1", "temp1".
     * 
     * @param temperature_reading_name 
     * @return double 
     */
    double get_cpu_temperature(const std::string& temperature_reading_name)
    {
        
    }
}