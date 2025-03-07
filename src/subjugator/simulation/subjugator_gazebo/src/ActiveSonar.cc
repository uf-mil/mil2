#include <gz/plugin/Register.hh>
#include "ActiveSonar.hh"
#include <iostream>

#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/laserscan.pb.h>

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

  std::string topic_pub = "/active_sonar/raw_data";
  gz::transport::Node node;
  auto pub = node.Advertise<gz::msgs::Twist>(topic_pub);

  std::string topic_sub = "active_sonar";

  std::function<void(const gz::msgs::Twist &)> callback;

  
  if (!node.Subscribe(topic_sub, callback))
  {
    std::cerr << "Error subscribing to topic for active_sonar." << std::endl;
  }

  // gz::transport::waitForShutdown();
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
