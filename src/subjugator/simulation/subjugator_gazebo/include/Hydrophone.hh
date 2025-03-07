#ifndef HYDROPHONE__HH_
#define HYDROPHONE__HH_

#include <chrono>
#include <gz/math/Pose3.hh>  // For gz::math::Pose3d
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/InstallationDirectories.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>  // For gz::sim::System
#include <gz/sim/World.hh>
#include <gz/sim/components/Name.hh>  // For gz::sim::components::Name
#include <gz/sim/components/Pose.hh>  // For gz::sim::components::Pose
#include <gz/transport/Node.hh>
#include <gz/utils/ImplPtr.hh>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sdf/Element.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/sdf.hh>
#include <string>
#include <vector>

#include "mil_msgs/msg/processed_ping.hpp"

namespace hydrophone
{
class Hydrophone : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
public:
  Hydrophone();
  ~Hydrophone();

  // System Configure
  void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

  // Gather World Info
  void GatherWorldInfo(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                       gz::sim::EntityComponentManager const &ecm);

  // System PostUpdate
  void PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm) override;

private:
  // Stored values of the hydrophone during Configure
  gz::sim::Entity modelEntity_{ gz::sim::kNullEntity };
  std::shared_ptr<sdf::Element const> sdf_;

  // ROS node + publisher
  std::shared_ptr<rclcpp::Node> rosNode_;
  rclcpp::Publisher<mil_msgs::msg::ProcessedPing>::SharedPtr pingPub_;

  // Boolean for GatherWorldInfo()
  bool worldGathered = false;

  // Hydrophone Pose
  gz::math::Pose3d pose_{ gz::math::Pose3d::Zero };

  // Pinger count
  int pingerCount_;

  // Pinger locations
  std::vector<gz::math::Pose3d> pingerLocations_;

  // Pinger frequency
  std::vector<double> pingerFrequencies_;

  // Pinger names
  std::vector<std::string> pingerNames_;

  // Difference Vectors - Where the pinger is relative to the hydrophone
  std::vector<gz::math::Vector3d> pingerDiffs_;
};

}  // namespace hydrophone
#endif  // HYDROPHONE_HH
