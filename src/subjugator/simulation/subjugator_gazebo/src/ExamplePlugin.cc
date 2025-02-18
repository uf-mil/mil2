#include <gz/plugin/Register.hh>
#include "ExamplePlugin.hh"
#include <iostream>

// Add plugins to gazebo simulator alongside its dependencies
GZ_ADD_PLUGIN(
    example_plugin::ExamplePlugin,
    gz::sim::System,
    example_plugin::ExamplePlugin::ISystemConfigure,
    example_plugin::ExamplePlugin::ISystemPostUpdate
)

using namespace example_plugin;

ExamplePlugin::ExamplePlugin()
{
}
 
ExamplePlugin::~ExamplePlugin()
{
}

void ExamplePlugin::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventManager)
{
  // std::cout << "init done!" << std::endl;
}
 
void ExamplePlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
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
