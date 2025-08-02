#ifndef TRUE_POSITION_HPP
#define TRUE_POSITION_HPP

#include <chrono>
#include <memory>
#include <mutex>
#include <random>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <gz/math/Pose3.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>
#include <mil_msgs/msg/true_position.hpp>
#include <sdf/sdf.hh>

namespace true_position
{

class TruePosition : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
  public:
    TruePosition();

    ~TruePosition() override = default;

    // System Configure
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    void PostUpdate(gz::sim::UpdateInfo const &_info, gz::sim::EntityComponentManager const &_ecm) override;

  private:
    std::shared_ptr<rclcpp::Node> ros_node_;
    gz::sim::Entity modelEntity_{ gz::sim::kNullEntity };

    // ROS node + publisher
    rclcpp::Publisher<mil_msgs::msg::TruePosition>::SharedPtr positionPub_;

    // Time bookkeeping in seconds
    double lastPubTime_{ 0.0 };
    double updatePeriod_{ 0.1 };  // default = 10 Hz

    std::string entityName;
};
}  // namespace true_position

#endif
