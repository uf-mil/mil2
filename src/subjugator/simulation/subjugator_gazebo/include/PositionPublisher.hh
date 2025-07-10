#ifndef POSITION_PUBLISHER_HH_
#define POSITION_PUBLISHER_HH_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Pose.hh>

namespace position_publisher
{
class PositionPublisher : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
  public:
    PositionPublisher() = default;
    ~PositionPublisher() override = default;

    // System Configure
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    void PostUpdate(gz::sim::UpdateInfo const &_info, gz::sim::EntityComponentManager const &_ecm) override;

  private:
    gz::sim::Model model{ gz::sim::kNullEntity };

    double X;
    double Y;
    double Z;
    double RotX;
    double RotY;
    double RotZ;
    double RotW;

    rclcpp::Node::SharedPtr rosNode;

    // ROS node + publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub;

    // Frame name
    std::string frameName_{ "position_publisher_frame" };
};
}  // namespace position_publisher

#endif
