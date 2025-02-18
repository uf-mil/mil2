#include <gz/plugin/Register.hh>
#include "ActiveSonar.hh"
#include <iostream>

// Add plugins to gazebo simulator alongside its dependencies
GZ_ADD_PLUGIN(
    active_sonar::ActiveSonar,
    gz::sim::System,
    active_sonar::ActiveSonar::ISystemConfigure,
    active_sonar::ActiveSonar::ISystemPostUpdate
)

using namespace active_sonar;

ActiveSonar::ActiveSonar()
{
}
 
ActiveSonar::~ActiveSonar()
{
}

void ActiveSonar::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventManager)
{
    //   std::cout << "init done on active sonar!" << std::endl;
}
 
void ActiveSonar::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  // Only runs code if the simulation is active
  if (!_info.paused)
  {
    //
    // Code placed in here will be called every update
    //

    // std::cout << "Hey there" << std::endl;
  }
}
