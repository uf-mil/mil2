#pragma once

#include <gz/msgs/double.pb.h>

#include <mutex>
#include <string>

#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace prop_gazebo
{
/// Differential thrust on a boat hull.
///
/// Two thrusters push along body x, one either side of the centreline, so
/// their sum is surge and their difference is yaw. Each side takes its thrust
/// in newtons on its own Gazebo topic:
///
///     <plugin filename="PropThrusters" name="prop_gazebo::PropThrusters">
///       <link_name>base_link</link_name>   <!-- hull to push -->
///       <thruster_y>0.25</thruster_y>      <!-- m off the centreline -->
///       <max_thrust>45.0</max_thrust>      <!-- N, per thruster -->
///       <topic_left>/prop/thrust/left</topic_left>
///       <topic_right>/prop/thrust/right</topic_right>
///     </plugin>
///
/// Gazebo ships a thruster system of its own, but it models a propeller: it
/// turns a thrust command into a propeller speed and the force back out of
/// that speed, and the newtons that come out do not match the newtons that
/// went in. A boat with two fixed thrusters does not need any of that.
class PropThrusters : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
  public:
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    void PreUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager &ecm) override;

  private:
    void OnLeft(gz::msgs::Double const &msg);
    void OnRight(gz::msgs::Double const &msg);

    gz::sim::Model model_{ gz::sim::kNullEntity };
    std::string link_name_{ "base_link" };
    double thruster_y_{ 0.25 };
    double max_thrust_{ 45.0 };

    gz::transport::Node node_;
    std::mutex mutex_;
    double left_{ 0.0 };
    double right_{ 0.0 };
};

}  // namespace prop_gazebo
