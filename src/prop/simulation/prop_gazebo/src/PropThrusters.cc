#include "PropThrusters.hh"

#include <algorithm>

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>

namespace prop_gazebo
{

void PropThrusters::Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                              gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &)
{
    this->model_ = gz::sim::Model(entity);
    if (!this->model_.Valid(ecm))
    {
        gzerr << "PropThrusters has to be attached to a model" << std::endl;
        return;
    }

    this->link_name_ = sdf->Get<std::string>("link_name", this->link_name_).first;
    this->thruster_y_ = sdf->Get<double>("thruster_y", this->thruster_y_).first;
    this->max_thrust_ = sdf->Get<double>("max_thrust", this->max_thrust_).first;

    auto const left = sdf->Get<std::string>("topic_left", "/prop/thrust/left").first;
    auto const right = sdf->Get<std::string>("topic_right", "/prop/thrust/right").first;
    this->node_.Subscribe(left, &PropThrusters::OnLeft, this);
    this->node_.Subscribe(right, &PropThrusters::OnRight, this);

    gzmsg << "PropThrusters pushing [" << this->model_.Name(ecm) << "::" << this->link_name_ << "] from [" << left
          << "] and [" << right << "]" << std::endl;
}

void PropThrusters::OnLeft(gz::msgs::Double const &msg)
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->left_ = std::clamp(msg.data(), -this->max_thrust_, this->max_thrust_);
}

void PropThrusters::OnRight(gz::msgs::Double const &msg)
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->right_ = std::clamp(msg.data(), -this->max_thrust_, this->max_thrust_);
}

void PropThrusters::PreUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager &ecm)
{
    if (info.paused)
    {
        return;
    }

    auto const entity = this->model_.LinkByName(ecm, this->link_name_);
    if (entity == gz::sim::kNullEntity)
    {
        gzerr << "PropThrusters found no link named [" << this->link_name_ << "]" << std::endl;
        return;
    }

    double left = 0.0;
    double right = 0.0;
    {
        std::lock_guard<std::mutex> lock(this->mutex_);
        left = this->left_;
        right = this->right_;
    }

    // Both thrusters push along body x. The wrench goes on in world axes, and
    // is applied at the centre of mass, so the yaw the offset thrusters would
    // have produced is added as a moment of its own.
    gz::math::Vector3d const force(left + right, 0.0, 0.0);
    gz::math::Vector3d const torque(0.0, 0.0, this->thruster_y_ * (right - left));

    auto const pose = gz::sim::worldPose(entity, ecm);
    gz::sim::Link link(entity);
    link.AddWorldWrench(ecm, pose.Rot() * force, pose.Rot() * torque);
}

}  // namespace prop_gazebo

GZ_ADD_PLUGIN(prop_gazebo::PropThrusters, gz::sim::System, prop_gazebo::PropThrusters::ISystemConfigure,
              prop_gazebo::PropThrusters::ISystemPreUpdate)
