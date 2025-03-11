#include "mil_tools/hardware/cpu_temp.hpp"
#include "mil_tools/hardware/system_info.hpp"
#include <sensors/sensors.h>
#include <cstring>
#include <string>

namespace mil_tools::hardware::cpu_temp
{
    double get_cpu_temperature()
    {
        // First we need to know the CPU architecture since the command/library we'll use for determining the temp differs based on the architecture
        std::string cpu_architecture = mil_tools::hardware::system_info::get_cpu_architecture();

        // If the architecture is x86, we'll use the lm-sensors/libsensors-cpp library
        if (cpu_architecture.compare("x86_64") == 0)
        {
            // Initialize sensors, none-zero returned value means there was an error during initialization
            if (sensors_init(nullptr) != 0) 
                // Return without modifying any values
                return -1;

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

                            if (strcmp(label, "temp1") == 0)
                                return temp;

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
            // To execute tegrastats, we can use a pipe and read the output from it
            FILE* pipe = popen("tegrastats --interval 0.1 | head -n 1 | grep -oP \'cpu@\\K[0-9]+\\.[0-9]+\'", "r");

            // We have to check if the pipe failed to execute before we read from it which will result in a nullptr
            if (!pipe)
                // If the pipe failed to execute, we can just return -1
                return -1;

            // We need a buffer to store the output of our command in chunks, as well as a string to combine everything
            char buffer[8];
            std::string tegrastats_output;

            // Reads the output from that pipe and appends it to a string
            while (fgets(buffer, 8, pipe) != NULL)
            tegrastats_output += buffer;

            // Now we can close the pipe and parse the string for the temperature reading
            pclose(pipe);

            // Parse the string to a double. I'll wrap it in a try-catch because
            // parsing has the potential to throw an exception
            try
            {
		        return(std::stod(tegrastats_output));
            }
            catch(const std::exception& e)
            {
                return -1;
            }

     	}

        // If the the get_cpu_architecture() function returns unknown, then we'll just return without modifying the cpu_temperatures object
        else
            return -1;

        return -1;
    }
}
