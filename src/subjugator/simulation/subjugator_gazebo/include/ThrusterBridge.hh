#pragma once

#include <iostream>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/InstallationDirectories.hh>
#include <gz/sim/System.hh>           // For gz::sim::System
#include <gz/sim/components/Name.hh>  // For gz::sim::components::Name
#include <gz/sim/components/Pose.hh>  // For gz::sim::components::Pose
#include <gz/transport/Node.hh>       // For gz::transport::Node
#include <sdf/Element.hh>

// The .msg file is named ProcessedPing.msg (PascalCase) but #include must be the name in snake_case
#include "subjugator_msgs/msg/thruster_efforts.hpp"

using std::placeholders::_1;

namespace thrusterBridge
{
class ThrusterBridge : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPostUpdate
{
  public:
    ThrusterBridge();
    ~ThrusterBridge();

    // Configure() -  Create ROS node that subscribes to /thruster_efforts & setup Gazebo Publishers //
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    // System PostUpdate - Run the ROS node //
    void PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm) override;

  private:
    // ROS node + publisher //
    rclcpp::Subscription<subjugator_msgs::msg::ThrusterEfforts>::SharedPtr thrustSubscription;
    std::shared_ptr<rclcpp::Node> thrustNode;

    // Gazebo Thruster Node //
    gz::transport::Node gz_node;

    // Callback for receiving thruster efforts //
    void receiveEffortCallback(subjugator_msgs::msg::ThrusterEfforts const &msg);

    // Thruster efforts storage - FLV = Front Left Vertical, BRH = Back Right Horizontal //
    std::unordered_map<std::string, float> thrusterEfforts;
    std::unordered_map<std::string, gz::transport::Node::Publisher> publishers;

    // Thruster Force Outputs //
    double max_force_pos = 1.0;
    double max_force_neg = 1.0;
};

}  // namespace thrusterBridge
