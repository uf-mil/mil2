#ifndef TORPEDOES__HH_
#define TORPEDOES__HH_

#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>  // For ROS2

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gz/plugin/Register.hh>  // For GZ_ADD_PLUGIN
// 'gz' and 'sdf' includes turn into namespace::namespace::ClassName
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
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

    // Configure() - Gathers Info at the start of the simulation //
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    // System PreUpdate - Called every simulation step before physics //
    void PreUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager &ecm) override;

    // System PostUpdate - Called every simulation step to update pinger/torpedoes distances //
    void PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm) override;

    // Callback for keypress
    void KeypressCallback(std_msgs::msg::String::SharedPtr const msg);

    // SpawnTorpedo - Spawns a torpedo in the simulation //
    void SpawnTorpedo(std::string const &worldName, std::string const &sdfPath);

  private:
    // ROS2 Node and Subscription for keypress //
    rclcpp::Node::SharedPtr torpedo_node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keypress_sub_;

    // Sub9 SDF & Entity //
    std::shared_ptr<sdf::Element const> sub9_SDF;
    gz::sim::Entity sub9_Entity{ gz::sim::kNullEntity };
    gz::math::Pose3d sub9_pose;
    bool subEntityFound = false;  // Track if the sub9 entity has been found

    // Torpedo Data Structures & Variables //
    std::set<std::string> torpedoModelNames;          // Unique names of torpedo models spawned
    std::set<std::string> torpedoesWithVelocitySet;   // Names of torpedoes that have had their velocity set
    std::map<std::string, double> torpedoSpawnTimes;  // Track spawn times of torpedoes
    int torpedoCount = 0;
    bool t_pressed = false;  // Track if 't' key was pressed
    bool worldNameFound = false;
    std::string worldName = "bingbong";

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
    // Pre-commit forces me to do this ugly sdfPath thingy above :(
};

}  // namespace torpedoes
#endif  // TORPEDOES_HH
