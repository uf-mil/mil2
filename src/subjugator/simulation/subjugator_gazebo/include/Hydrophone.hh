#ifndef HYDROPHONE_HPP
#define HYDROPHONE_HPP

#include <chrono>
#include <gz/math/Pose3.hh>           // For gz::math::Pose3d
#include <gz/sim/System.hh>           // For gz::sim::System
#include <gz/sim/components/Pose.hh>  // For gz::sim::components::Pose
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>
#include <string>

#include "mil_msgs/msg/processed_ping.hpp"

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
  // Message Type: Vector3d origin_direction_body
  //                int frequency
  //                float origin_distance_m
  rclcpp::Node::SharedPtr rosNode_;
  rclcpp::Publisher<mil_msgs::msg::ProcessedPing>::SharedPtr pingPub_;

  // Hydrophone Pose
  gz::math::Pose3d pose_{ gz::math::Pose3d::Zero };

  // Pinger count
  int pingerCount_;

  // Pinger locations
  std::vector<gz::math::Pose3d> pingerLocations_;

  // Pinger frequency
  std::vector<double> pingerFrequencies_;

  // Difference Vectors - Where the pinger is relative to the hydrophone
  std::vector<gz::math::Vector3d> pingerDiffs_;
};

#endif
