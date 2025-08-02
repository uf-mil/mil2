#include "TruePosition.hh"

#include <gz/msgs/pose.pb.h>

#include <rclcpp/rclcpp.hpp>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>  // For GZ_ADD_PLUGIN
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

GZ_ADD_PLUGIN(true_position::TruePosition, gz::sim::System, true_position::TruePosition::ISystemConfigure,
              true_position::TruePosition::ISystemPostUpdate)

namespace true_position
{

TruePosition::TruePosition()
{
}

void TruePosition::Configure(gz::sim::Entity const &_entity, std::shared_ptr<sdf::Element const> const &_sdf,
                             gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventManager)
{
    this->modelEntity_ = _entity;

    auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
    if (nameComp)
    {
        this->entityName = nameComp->Data();
    }
    else
    {
        this->entityName = "unknown_entity_" + std::to_string(_entity);
    }

    // Grab our update rate from the xacro file
    if (_sdf->HasElement("update_rate"))
    {
        float rate = _sdf->Get<float>("update_rate");
        if (rate > 0.0)
        {
            this->updatePeriod_ = 1.0 / rate;
        }
    }

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    this->ros_node_ = std::make_shared<rclcpp::Node>("true_position_node");
    this->positionPub_ = this->ros_node_->create_publisher<mil_msgs::msg::TruePosition>("/true_position", 10);
}

void TruePosition::PostUpdate(gz::sim::UpdateInfo const &_info, gz::sim::EntityComponentManager const &_ecm)
{
    if (!_info.paused)
    {
        rclcpp::spin_some(this->ros_node_);

        // Convert simTime (steady_clock::duration) to float (seconds)
        float nowSec = std::chrono::duration_cast<std::chrono::duration<float>>(_info.simTime).count();

        float dt = nowSec - this->lastPubTime_;

        // Haven't met our update threshold yet
        if (dt < this->updatePeriod_)
            return;

        this->lastPubTime_ = nowSec;

        // Get the pose component which has our position data
        auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->modelEntity_);
        if (!poseComp)
        {
            gzerr << "[TruePosition] Pose component not found for entity [" << this->modelEntity_ << "]\n";
            return;
        }

        auto const &pose = poseComp->Data();

        // Create a msg object to contain our position/orientation data, then publish it
        mil_msgs::msg::TruePosition msg;
        msg.position = { pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() };
        msg.orientation = { pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw() };

        this->positionPub_->publish(msg);
    }
}
}  // namespace true_position
