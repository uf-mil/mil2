#ifndef THRUSTERBRIDGE__HH_
#define THRUSTERBRIDGE__HH_

#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <gz/math/Pose3.hh>  // For gz::math::Pose3d
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/InstallationDirectories.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>  // For gz::sim::System
#include <gz/sim/World.hh>
#include <gz/sim/components/Name.hh>  // For gz::sim::components::Name
#include <gz/sim/components/Pose.hh>  // For gz::sim::components::Pose
#include <gz/transport/Node.hh>       // For gz::transport::Node
#include <sdf/Element.hh>
#include <sdf/World.hh>
#include <sdf/sdf.hh>

// The .msg file is named ProcessedPing.msg (PascalCase) but #include must be the name in snake_case
#include "subjugator_msgs/msg/thruster_efforts.hpp"

using std::placeholders::_1;

namespace thrusterBridge
{
class ThrusterBridge : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate

{
  public:
    ThrusterBridge();
    ~ThrusterBridge();

    // Configure() -  //
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    // System PostUpdate - //
    void PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm) override;

  private:
    // ROS node + publisher //
    rclcpp::Publisher<subjugator_msgs::msg::ThrusterEfforts>::SharedPtr thrustPublisher;
    rclcpp::Subscription<subjugator_msgs::msg::ThrusterEfforts>::SharedPtr thrustSubscription;
    std::shared_ptr<rclcpp::Node> thrustNode;

    // Gazebo Thruster Node //
    gz::transport::Node gz_node;

    // Callback for receiving thruster efforts //
    void receiveEffortCallback(subjugator_msgs::msg::ThrusterEfforts const &msg);
    void receiveGazeboCallback(gz::msgs::Double const &msg);

    // Thruster efforts storage - FLV = Front Left Vertical, BRH = Back Right Horizontal //
    std::vector<std::string> thrusterNames{ flh, frh, blh, brh, flv, frv, blv, brv };
    float flh;
    float frh;
    float blh;
    float brh;
    float flv;
    float frv;
    float blv;
    float brv;
};

}  // namespace thrusterBridge
#endif  // THRUSTERBRIDGE__HH_
