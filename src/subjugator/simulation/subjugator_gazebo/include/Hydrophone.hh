#ifndef HYDROPHONE_HPP
#define HYDROPHONE_HPP

#include <chrono>
#include <gz/math/Pose3.hh>           // For gz::math::Pose3d
#include <gz/sim/System.hh>           // For gz::sim::System
#include <gz/sim/components/Pose.hh>  // For gz::sim::components::Pose
#include <memory>
#include <mil_msgs/msg/depth_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>
#include <string>

namespace hydrophone
{

class Hydrophone : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
public:
  Hydrophone();
  ~Hydrophone() override;

  // System Configure
  void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) /* no override */;

  // System PostUpdate
  void PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm) /* no override */;

private:
  gz::sim::Entity modelEntity_{ gz::sim::kNullEntity };

  // ROS node + publisher
  // rclcpp::Node::SharedPtr rosNode_;
  // rclcpp::Publisher<mil_msgs::msg::DepthStamped>::SharedPtr depthPub_;

  // Hydrophone Pose
  gz::math::Pose3d pose_{ gz::math::Pose3d::Zero };

  // // Pose offset
  // gz::math::Pose3d offset_{gz::math::Pose3d::Zero};

  // // Frame name
  // std::string frameName_{"depth_sensor_frame"};

  // // Time bookkeeping in seconds
  // double lastPubTime_{0.0};
  // double updatePeriod_{0.1}; // default = 10 Hz
};

}  // namespace hydrophone

#endif
