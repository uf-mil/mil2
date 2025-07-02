#ifndef TORPEDOES__HH_
#define TORPEDOES__HH_

#include <gz/msgs/entity_factory.pb.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gz/math/Pose3.hh>    // For gz::math::Pose3d
#include <gz/math/Vector3.hh>  // For gz::math::Vector3d
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/InstallationDirectories.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>  // For gz::sim::System
#include <gz/sim/World.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/CanonicalLink.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>  // For gz::sim::components::Name
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>  // For gz::sim::components::Pose
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
#include <sdf/World.hh>
#include <sdf/sdf.hh>

// Insert ROS Message Here if needed

namespace torpedoes
{
class Torpedoes : public gz::sim::System,
                  public gz::sim::ISystemConfigure,
                  public gz::sim::ISystemPreUpdate,
                  public gz::sim::ISystemPostUpdate
{
  public:
    Torpedoes();
    ~Torpedoes();

    // Configure() - Gathers Info at the start of the simulation //
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    // SpawnTorpedo - Spawns a torpedo in the simulation //
    void SpawnTorpedo(std::string const &worldName, std::string const &sdfPath);

    // System PreUpdate - Called every simulation step before physics //
    void PreUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager &ecm) override;

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

    // Torpedo SDF Values //
    unsigned int timeout = 2000;  // Timeout in milliseconds
    gz::msgs::Boolean reply;
    bool result = false;
    gz::transport::Node node;
    std::string const Torpedo_sdfPath = ament_index_cpp::get_package_share_directory("subjugator_description") + "/mode"
                                                                                                                 "ls/"
                                                                                                                 "torpe"
                                                                                                                 "do/"
                                                                                                                 "torpe"
                                                                                                                 "do."
                                                                                                                 "sdf";

    // Track torpedo model names
    std::set<std::string> torpedoModelNames;

    // Track torpedoes that have already had their velocity set
    std::set<std::string> torpedoesWithVelocitySet;
    // Track attempts to set velocity for each torpedo
    std::map<std::string, int> torpedoVelocitySetAttempts;
};

}  // namespace torpedoes
#endif  // TORPEDOES_HH
