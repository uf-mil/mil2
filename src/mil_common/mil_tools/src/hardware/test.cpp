#include <iostream>
#include <sensors/sensors.h>

using std::cout;

int main() 
{
    // Initialize sensors, none-zero returned value means there was an error during initialization
    if (sensors_init(nullptr) != 0) 
    {
        std::cerr << "Failed to initialize sensors" << std::endl;
        return 1;
    }

    // Iterate through all chips
    const sensors_chip_name *chip;
    
    // Specify starting chip as 0 so that we go through all available chips
    int chip_nr = 0;
    
    // Pass a nullptr for the sensors_chip_name argument to tell the function to read all available chips
    // since we might change our CPU or components.
    while ((chip = sensors_get_detected_chips(nullptr, &chip_nr)) != nullptr) 
    {
	std::cout << "Entered loop\n";
        // Get all features (sensors) of the chip
        const sensors_feature *feature;
        int feature_nr = 0;
        
        while ((feature = sensors_get_features(chip, &feature_nr))) 
        {
	    std::cout << "Entered second loop" << std::endl;
            // We only care about temperature readings so any feature that's not temp we can skip
            /*if (feature->type != SENSORS_FEATURE_TEMP) 
            {
                continue;
            }*/

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

                    cout << label << ": " << temp << "\n";

                    // Free the string
                    free(label);
                }
            }
        }
    }

    sensors_cleanup();
    return 0;
}
