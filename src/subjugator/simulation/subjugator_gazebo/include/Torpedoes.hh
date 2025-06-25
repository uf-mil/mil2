#ifndef TORPEDOES__HH_
#define TORPEDOES__HH_

#include <gz/msgs/entity_factory.pb.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gz/math/Pose3.hh>  // For gz::math::Pose3d
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/InstallationDirectories.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>  // For gz::sim::System
#include <gz/sim/World.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>  // For gz::sim::components::Name
#include <gz/sim/components/Pose.hh>  // For gz::sim::components::Pose
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
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

    // SpawnTorpedo - Spawns a torpedo in the simulation //
    void SpawnTorpedo(std::string const &worldName, std::string const &sdfPath);

    // System PostUpdate - Called every simulation step to update pinger/torpedoes distances //
    void PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm) override;

  private:
    // Sub9 SDF & Entity //
    std::shared_ptr<sdf::Element const> sub9_SDF;
    gz::sim::Entity sub9_Entity{ gz::sim::kNullEntity };

    // Torpedo Variables //
    int torpedoCount = 0;
    bool worldNameFound = false;
    std::string worldName = "task1_2025.world";

    // Torpedo SDF Filepath //
    std::string const Torpedo_sdfPath = ament_index_cpp::get_package_share_directory("subjugator_description") + "/mode"
                                                                                                                 "ls/"
                                                                                                                 "torpe"
                                                                                                                 "do/"
                                                                                                                 "torpe"
                                                                                                                 "do."
                                                                                                                 "sdf";
};

}  // namespace torpedoes
#endif  // TORPEDOES_HH
