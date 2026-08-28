#ifndef DEPTH_SENSOR_HPP
#define DEPTH_SENSOR_HPP

#include <chrono>
#include <memory>
#include <mutex>
#include <random>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <gz/math/Pose3.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>
#include <mil_msgs/msg/depth_stamped.hpp>
#include <sdf/sdf.hh>

namespace depth_sensor
{

class DepthSensor : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
  public:
    DepthSensor();

    ~DepthSensor() override = default;

    // System Configure
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    void PostUpdate(gz::sim::UpdateInfo const &_info, gz::sim::EntityComponentManager const &_ecm) override;

  private:
    std::shared_ptr<rclcpp::Node> ros_node_;
    gz::sim::Entity modelEntity_{ gz::sim::kNullEntity };

    // ROS node + publisher
    rclcpp::Publisher<mil_msgs::msg::DepthStamped>::SharedPtr depthPub_;

    // Pose offset
    gz::math::Pose3d offset_{ gz::math::Pose3d::Zero };

    // Frame name
    std::string frameName_{ "depth_sensor_frame" };

    // Time bookkeeping in seconds
    double lastPubTime_{ 0.0 };
    double updatePeriod_{ 0.1 };  // default = 10 Hz

    double noiseMean_ = 0.0;
    double noiseStdDev_ = 0.0;  // default to 0 => no noise if not specified
    std::mt19937 randomEngine_;
    // NOTE: stddev must be > 0 -- std::normal_distribution's ctor asserts on
    // stddev <= 0 with modern libstdc++. A zero-noise sensor is expressed by
    // noiseStdDev_ == 0.0 and guarded at the sampling site, not by a zero-stddev
    // distribution. The 1.0 here is just a valid placeholder; it is only ever
    // sampled when noiseStdDev_ > 0 (in which case it is reassigned below).
    std::normal_distribution<double> noiseDist_{ 0.0, 1.0 };
};
}  // namespace depth_sensor

#endif
