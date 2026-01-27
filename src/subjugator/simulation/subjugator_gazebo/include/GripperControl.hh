#ifndef GRIPPER_CONTROL_HH_
#define GRIPPER_CONTROL_HH_

#include <gz/msgs/double.pb.h>

#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
#include <std_msgs/msg/string.hpp>

namespace gripper_control
{

class GripperControl : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
  public:
    GripperControl();
    ~GripperControl() override;

    // Configure - called during simulation start
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    // PostUpdate - called each simulation step
    void PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm) override;

    // Keypress callback
    void KeypressCallback(std_msgs::msg::String::SharedPtr const msg);

  private:
    // ROS2 Node and subscription
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;

    // Gazebo transport node + publisher
    gz::transport::Node gz_node_;
    gz::transport::Node::Publisher gz_pub_;

    // State & settings
    bool u_pressed_{ false };
    bool gripper_open_{ false };
    double open_pos_{ 0.5 };    // radians (default)
    double closed_pos_{ 0.0 };  // radians (default)
    std::string joint_name_{ "gripper_joint" };
    std::string model_name_{ "" };
    std::string topic_name_{ "" };

    // World name (for nicer logging)
    std::string world_name_{ "unknown" };
};

}  // namespace gripper_control

#endif  // GRIPPER_CONTROL_HH_
