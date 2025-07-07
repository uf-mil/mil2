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
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>    // For gz::math::Pose3d
#include <gz/math/Vector3.hh>  // For gz::math::Vector3d
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/InstallationDirectories.hh>
#include <gz/sim/Link.hh>
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
#include <std_msgs/msg/string.hpp>

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

    // Configure() - Stores sub9 information & creates ros2 functionality before simulation starts //
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    // System PreUpdate - Assigns velocity to any spawned torpedoes //
    void PreUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager &ecm) override;

    // System PostUpdate - Handles updating sub9 pose & torpedo spawning functionality //
    void PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm) override;

  private:
    // Callback for ROS2 keypress subscription //
    void KeypressCallback(std_msgs::msg::String::SharedPtr const msg);

    // SpawnTorpedo - Spawns a torpedo with unique name & relative to sub9 position //
    void SpawnTorpedo(std::string const &worldName, std::string const &sdfPath);

    // ROS2 Node and Subscription for keypress //
    rclcpp::Node::SharedPtr torpedo_node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keypress_sub_;

    // Sub9 Information //
    std::shared_ptr<sdf::Element const> sub9_SDF;
    gz::sim::Entity sub9_Entity{ gz::sim::kNullEntity };
    gz::math::Pose3d sub9_pose;
    bool subEntityFound = false;  // Track if the sub9 entity has been found

    // Torpedo Variables //
    int torpedoCount = 0;
    bool t_pressed = false;  // Track if 't' key was pressed
    bool worldNameFound = false;
    std::string worldName = "task1_2025.world";

    // Torpedo Data Structures //
    std::set<std::string> torpedoModelNames;         // Names of Spawned Torpedo Models
    std::set<std::string> torpedoesWithVelocitySet;  // Which torpedoes have had their velocity set

    // Spawning Torpedoes Information //
    unsigned int timeout = 2000;  // Timeout in milliseconds
    gz::msgs::Boolean reply;
    bool result = false;
    gz::transport::Node node;
    std::string const Torpedo_sdfPath = ament_index_cpp::get_package_share_directory("subjugator_description") + "/mode"
                                                                                                                 "ls/"
                                                                                                                 "torpe"
                                                                                                                 "does/"
                                                                                                                 "torpe"
                                                                                                                 "do."
                                                                                                                 "sdf";
    // idk why this is so ugly, pre-commit keeps forcing it to look like this
};

}  // namespace torpedoes

#endif  // TORPEDOES_HH
