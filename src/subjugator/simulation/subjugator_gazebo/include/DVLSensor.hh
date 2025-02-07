#ifndef DVL_SENSOR_HH_
#define DVL_SENSOR_HH_

#include <gz/sim/System.hh>
 
namespace dvl_sensor
{
  class DVLSensor:
    // Defines that this class is a system
    public gz::sim::System,

    // Implements the ISystemPostUpdate interface for sensor updates
    public gz::sim::ISystemPostUpdate
  {
    public: DVLSensor();
 
    public: ~DVLSensor() override;
 
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
  };
 

}

#endif