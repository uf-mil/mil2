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
/// Fixed thrusters on a boat hull.
///
/// Two push along body x, one either side of the centreline, so their sum is
/// surge and their difference is yaw. A third pushes along body y ahead of the
/// centre of mass, for sway and a little more yaw. Each takes its thrust in
/// newtons on its own Gazebo topic:
///
///     <plugin filename="PropThrusters" name="prop_gazebo::PropThrusters">
///       <link_name>base_link</link_name>   <!-- hull to push -->
///       <thruster_y>0.27305</thruster_y>   <!-- m off the centreline -->
///       <max_force_pos>55.21</max_force_pos>  <!-- N per aft thruster, ahead -->
///       <max_force_neg>27.56</max_force_neg>  <!-- N per aft thruster, astern -->
///       <bow_x>0.4</bow_x>                    <!-- m ahead of the centre of mass -->
///       <bow_force_pos>51.5</bow_force_pos>   <!-- N, to port -->
///       <bow_force_neg>40.2</bow_force_neg>   <!-- N, to starboard -->
///       <topic_left>/prop/thrust/left</topic_left>
///       <topic_right>/prop/thrust/right</topic_right>
///       <topic_bow>/prop/thrust/bow</topic_bow>
///     </plugin>
///
/// Gazebo ships a thruster system of its own, but it models a propeller: it
/// turns a thrust command into a propeller speed and the force back out of
/// that speed, and the newtons that come out do not match the newtons that
/// went in. A boat with fixed thrusters does not need any of that.
class PropThrusters : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
  public:
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    void PreUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager &ecm) override;

  private:
    void OnLeft(gz::msgs::Double const &msg);
    void OnRight(gz::msgs::Double const &msg);
    void OnBow(gz::msgs::Double const &msg);

    gz::sim::Model model_{ gz::sim::kNullEntity };
    std::string link_name_{ "base_link" };
    double thruster_y_{ 0.27305 };
    double max_force_pos_{ 55.21 };
    double max_force_neg_{ 27.56 };
    double bow_x_{ 0.4 };
    double bow_force_pos_{ 51.5 };
    double bow_force_neg_{ 40.2 };

    gz::transport::Node node_;
    std::mutex mutex_;
    double left_{ 0.0 };
    double right_{ 0.0 };
    double bow_{ 0.0 };
};

}  // namespace prop_gazebo
