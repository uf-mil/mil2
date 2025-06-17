#ifndef TORPEDOES__HH_
#define TORPEDOES__HH_

#include <chrono>
#include <iostream>
#include <string>
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
#include <sdf/Element.hh>
#include <sdf/World.hh>
#include <sdf/sdf.hh>

// The .msg file is named ProcessedPing.msg (PascalCase) but #include must be the name in snake_case
// Insert ROS Message Here if needed

namespace torpedoes
{
class Torpedoes : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
  public:
    Torpedoes();
    ~Torpedoes();

    // Configure() - Gathers Info at the start of the simulation //
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    // System PostUpdate - Called every simulation step to update pinger/torpedoes distances //
    void PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm) override;

  private:
    // Stored Entities //
    gz::sim::Entity modelEntity_{ gz::sim::kNullEntity };      // World Entity
    gz::sim::Entity torpedoesEntity_{ gz::sim::kNullEntity };  // Torpedoes Entity

    // Sub9 SDF //
    std::shared_ptr<sdf::Element const> sub9_SDF;
};

}  // namespace torpedoes
#endif  // TORPEDOES_HH
