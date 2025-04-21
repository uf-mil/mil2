#include "mil_tools/hardware/cpu_temp.hpp"

#include <sensors/sensors.h>

#include <cstring>
#include <string>

#include "mil_tools/hardware/system_info.hpp"

namespace mil_tools::hardware::cpu_temp
{
double get_cpu_temperature()
{
    // First we need to know the CPU architecture since the command/library we'll use for determining the temp differs
    // based on the architecture
    std::string cpu_architecture = mil_tools::hardware::system_info::get_cpu_architecture();

    // If the architecture is x86, we'll use the lm-sensors/libsensors-cpp library
    if (cpu_architecture.compare("x86_64") == 0)
    {
        // Initialize sensors, none-zero return means there was an error during initialization
        if (sensors_init(nullptr) != 0)
            return -1;

        // Chip object we can use to extract data about the device
        sensors_chip_name const *chip;

        // Specify starting chip as 0 so that we go through all available chips
        int chip_nr = 0;

        // Pass a nullptr for the sensors_chip_name argument to tell the function to read all available chips
        // since we might change our CPU or components.
        while ((chip = sensors_get_detected_chips(nullptr, &chip_nr)) != nullptr)
        {
            // Get all features (sensors) of the chip
            sensors_feature const *feature;
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
                sensors_subfeature const *subfeature =
                    sensors_get_subfeature(chip, feature, SENSORS_SUBFEATURE_TEMP_INPUT);

                if (subfeature)
                {
                    if (sensors_get_value(chip, subfeature->number, &temp) == 0)
                    {
                        // Get the name of the sensor
                        char *label = sensors_get_label(chip, feature);

                        if (strcmp(label, "Composite") == 0)
                            return temp;

                        free(label);
                    }

                    // If we reach this point in the code, that means the "Composite" reading couldn't be found
                    // so we'll need to try a different temp sensor
                    if (sensors_get_value(chip, subfeature->number, &temp) == 0)
                    {
                        char *label = sensors_get_label(chip, feature);

                        if (strcmp(label, "temp1") == 0)
                            return temp;

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
        FILE *pipe = popen("tegrastats --interval 0.1 | head -n 1 | grep -oP \'cpu@\\K[0-9]+\\.[0-9]+\'", "r");

        // We have to check if the pipe failed to execute before we read from it which will result in a nullptr
        if (!pipe)
            return -1;

        char buffer[8];
        std::string tegrastats_output;

        // Read the output
        while (fgets(buffer, 8, pipe) != NULL)
            tegrastats_output += buffer;

        pclose(pipe);

        try
        {
            return (std::stod(tegrastats_output));
        }
        catch (std::exception const &e)
        {
            return -1;
        }
    }

    return -1;
}
}  // namespace mil_tools::hardware::cpu_temp
