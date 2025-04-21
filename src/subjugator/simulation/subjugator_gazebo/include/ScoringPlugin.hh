#ifndef SCORING_PLUGIN_HH
#define SCORING_PLUGIN_HH

#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

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
    void CheckForStylePoints(gz::math::Pose3d const &currentPose);
    void ResetStyleTracking();
    int CalculateStylePoints();
    double angleChange(double previous, double current);

  private:
    gz::sim::Entity subEntity_{ gz::sim::kNullEntity };  // this plugin's parent model
    gz::sim::Entity gateEntity_{ gz::sim::kNullEntity };
    std::string gateModelName_;

    // Score tracking
    int score_{ 0 };
    bool gatePassed_{ false };

    // Checking for rotations in gate
    bool inGateStyleZone_{ false };
    gz::math::Pose3d lastRecordedPose_;
    gz::math::Pose3d initialPose_;

    struct OrientationRecord
    {
        double roll{ 0.0 };
        double pitch{ 0.0 };
        double yaw{ 0.0 };
        bool rollChanged{ false };
        bool pitchChanged{ false };
        bool yawChanged{ false };
        int rollCounter{ 0 };
        int pitchCounter{ 0 };
        int yawCounter{ 0 };
        double lastRoll{ 0.0 };
        double lastPitch{ 0.0 };
        double lastYaw{ 0.0 };
        double accumulatedRoll{ 0.0 };
        double accumulatedPitch{ 0.0 };
        double accumulatedYaw{ 0.0 };
    };

    OrientationRecord styleRecord_;

    int const ROLL_POINTS = 30;
    int const PITCH_POINTS = 25;
    int const YAW_POINTS = 15;

    // Will not consider orientation change until it passes this many degrees
    double const ORIENTATION_THRESHOLD = 80.0;

    rclcpp::Time styleStartTime_;
    bool styleTimerActive_{ false };
    bool timeLimitReached_{ false };
    // Picked a high number for testing
    double const STYLE_TIME_LIMIT = 60.0;  // seconds

    rclcpp::Node::SharedPtr rosNode_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr scorePub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr eventPub_;
};
}  // namespace scoring_plugin

#endif
