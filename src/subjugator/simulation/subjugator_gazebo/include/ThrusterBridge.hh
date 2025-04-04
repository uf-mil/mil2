#ifndef HYDROPHONE__HH_
#define HYDROPHONE__HH_

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <gz/math/Pose3.hh>  // For gz::math::Pose3d
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/InstallationDirectories.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>  // For gz::sim::System
#include <gz/sim/World.hh>
#include <gz/sim/components/Name.hh>  // For gz::sim::components::Name
#include <gz/sim/components/Pose.hh>  // For gz::sim::components::Pose
#include <sdf/Element.hh>
#include <sdf/World.hh>
#include <sdf/sdf.hh>

namespace hydrophone
{
class Hydrophone : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
  public:
    Hydrophone();
    ~Hydrophone();

    // Configure() - Gathers Info at the start of the simulation //
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    // FormatPinger() - Extract frequency and name //
    void FormatPinger(std::string &entityName);

    // System PostUpdate - Called every simulation step to update pinger/hydrophone distances //
    void PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm) override;

    // GatherWorldInfo() - Scrape info from the worldSDF to get pinger info //
    void GatherWorldInfo(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                         gz::sim::EntityComponentManager const &ecm);

  private:
    // Boolean so GatherWorldInfo is only called once //
    bool worldGathered = false;

    // Stored Entities //
    gz::sim::Entity modelEntity_{ gz::sim::kNullEntity };       // World Entity
    gz::sim::Entity hydrophoneEntity_{ gz::sim::kNullEntity };  // Hydrophone Entity

    // World SDF //
    std::shared_ptr<sdf::Element const> sdf_;

    // ROS node + publisher //
    std::shared_ptr<rclcpp::Node> rosNode_;
    rclcpp::Publisher<mil_msgs::msg::ProcessedPing>::SharedPtr pingPub_;

    // Vectors for Pinger Info //
    std::vector<std::string> pingerNames_;
    std::vector<gz::math::Pose3d> pingerLocations_;
    std::vector<double> pingerFrequencies_;

    // Sub9 Pose //
    gz::math::Pose3d hydrophonePose_{ gz::math::Pose3d::Zero };

    // Difference Vectors - Where the pinger is relative to the hydrophone
    std::vector<gz::math::Vector3d> pingerDiffs_;
};

}  // namespace hydrophone
#endif  // HYDROPHONE_HH
