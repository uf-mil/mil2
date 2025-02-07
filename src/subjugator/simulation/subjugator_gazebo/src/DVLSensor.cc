#include <gz/plugin/Register.hh>
#include "DVLSensor.hh"

#include <iostream>

// Add plugins to gazebo simulator alongside its dependencies
GZ_ADD_PLUGIN(
    dvl_sensor::DVLSensor,
    gz::sim::System,
    dvl_sensor::DVLSensor::ISystemPostUpdate
)

using namespace dvl_sensor;

DVLSensor::DVLSensor()
{
}
 
DVLSensor::~DVLSensor()
{
}
 
void DVLSensor::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  // Only runs code if the simulation is active
  if (!_info.paused)
  {
    //
    // Code placed in here will be called every update
    //

    std::cout << "Hey there" << std::endl;
  }
}
