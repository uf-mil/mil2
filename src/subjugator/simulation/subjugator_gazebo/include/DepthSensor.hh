#ifndef DEPTH_SENSOR_HPP
#define DEPTH_SENSOR_HPP

#include <mil_msgs/msg/depth_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gz/sim/System.hh>             // For gz::sim::System
#include <gz/sim/components/Pose.hh>    // For gz::sim::components::Pose
#include <gz/math/Pose3.hh>             // For gz::math::Pose3d
#include <sdf/sdf.hh>

#include <string>
#include <memory>
#include <chrono>

namespace depth_sensor
{ 

class DepthSensor 
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  
  DepthSensor();
  ~DepthSensor() override;

  // System Configure
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) /* no override */;

  // System PostUpdate
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) /* no override */;

private:
  gz::sim::Entity modelEntity_{gz::sim::kNullEntity};

  // ROS node + publisher
  rclcpp::Node::SharedPtr rosNode_;
  rclcpp::Publisher<mil_msgs::msg::DepthStamped>::SharedPtr depthPub_;

  // Pose offset
  gz::math::Pose3d offset_{gz::math::Pose3d::Zero};

  // Frame name
  std::string frameName_{"depth_sensor_frame"};

  // Time bookkeeping in seconds
  double lastPubTime_{0.0};
  double updatePeriod_{0.1}; // default = 10 Hz
};

} // namespace depth_sensor

#endif
