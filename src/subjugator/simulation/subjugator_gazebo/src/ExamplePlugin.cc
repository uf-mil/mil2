#include "ExamplePlugin.hh"

#include <gz/plugin/Register.hh>
#include <iostream>

// Add plugins to gazebo simulator alongside its dependencies
GZ_ADD_PLUGIN(dvl_sensor::ExamplePlugin, gz::sim::System, dvl_sensor::ExamplePlugin::ISystemPostUpdate)

using namespace dvl_sensor;

ExamplePlugin::ExamplePlugin()
{
}

ExamplePlugin::~ExamplePlugin()
{
}

void ExamplePlugin::Configure(gz::sim::Entity const &_entity, std::shared_ptr<sdf::Element const> const &_sdf,
                              gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventManager)
{
  // std::cout << "init done!" << std::endl;
}

void ExamplePlugin::PostUpdate(gz::sim::UpdateInfo const &_info, gz::sim::EntityComponentManager const &_ecm)
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
