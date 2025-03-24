#ifndef SCORING_PLUGIN_HH
#define SCORING_PLUGIN_HH

#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace scoring_plugin
{
class ScoringPlugin : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
  public:
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    void PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm) override;

  private:
    bool SubPassedThroughGate(gz::math::Pose3d const &subPose, gz::math::Pose3d const &gatePose);
    double getAngleBetweenVectors(gz::math::Pose3d const &subPose, gz::math::Pose3d const &gatePose);

  private:
    // The submarine’s entity (this plugin’s parent model)
    gz::sim::Entity subEntity_{ gz::sim::kNullEntity };

    // The gate’s entity, found by name
    gz::sim::Entity gateEntity_{ gz::sim::kNullEntity };

    // The gate’s model name (passed in via SDF <gate_name>)
    std::string gateModelName_;

    // Score tracking
    int score_{ 0 };
    bool gatePassed_{ false };

    // ROS
    rclcpp::Node::SharedPtr rosNode_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr scorePub_;
};
}  // namespace scoring_plugin

#endif
