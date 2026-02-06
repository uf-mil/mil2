#ifndef GRIPPER_CONTROL_HH_
#define GRIPPER_CONTROL_HH_

#include <gz/msgs/double.pb.h>

#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
#include <std_msgs/msg/string.hpp>

namespace gripper_control
{

class GripperControl : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate,
                       public gz::sim::ISystemPostUpdate
{
  public:
    GripperControl();
    ~GripperControl() override;

    // Configure - called during simulation start
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    // PreUpdate - called each simulation step (read-write access)
    void PreUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager &_ecm) override;

    // PostUpdate - called each simulation step (read-only access)
    void PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm) override;

    // Keypress callback
    void KeypressCallback(std_msgs::msg::String::SharedPtr const msg);

  private:
    // ROS2 Node and subscription
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;

    // Gazebo transport node + publishers for left and right joints
    gz::transport::Node gz_node_;
    gz::transport::Node::Publisher gz_pub_left_;
    gz::transport::Node::Publisher gz_pub_right_;

    // State & settings
    bool u_pressed_{ false };
    bool gripper_open_{ false };
    double open_pos_{ 0.79 };   // radians (default)
    double closed_pos_{ 0.0 };  // radians (default)
    // Base joint name (optional); specific left/right names computed from this if provided
    std::string joint_name_{ "gripper" };
    std::string left_joint_name_{ "gripper_leftArm_joint" };
    std::string right_joint_name_{ "gripper_rightArm_joint" };
    // std::string left_topic_{ "" };
    // std::string right_topic_{ "" };
    std::string model_name_{ "" };
    // std::string topic_name_{ "" };

    // Cached joint entities for direct control
    gz::sim::Entity left_joint_entity_{ gz::sim::kNullEntity };
    gz::sim::Entity right_joint_entity_{ gz::sim::kNullEntity };

    // Smooth motion state: targets and current positions
    double left_target_pos_{ 0.0 };
    double right_target_pos_{ 0.0 };
    double left_current_pos_{ 0.0 };
    double right_current_pos_{ 0.0 };
    double smoothing_alpha_{ 0.2 };  // fractional step toward target each PreUpdate
    bool left_pos_initialized_{ false };
    bool right_pos_initialized_{ false };

    // World name (for nicer logging)
    std::string world_name_{ "unknown" };
};

}  // namespace gripper_control

#endif  // GRIPPER_CONTROL_HH_
