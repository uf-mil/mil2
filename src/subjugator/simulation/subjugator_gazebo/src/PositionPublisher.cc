#include "PositionPublisher.hh"

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>

// Register plugin for Gazebo
GZ_ADD_PLUGIN(position_publisher::PositionPublisher, gz::sim::System, gz::sim::ISystemConfigure,
              gz::sim::ISystemPostUpdate)
namespace position_publisher
{
void PositionPublisher::Configure(gz::sim::Entity const &_entity, std::shared_ptr<sdf::Element const> const &_sdf,
                                  gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventManager)
{
    this->model = gz::sim::Model(_entity);
    this->rosNode = std::make_shared<rclcpp::Node>("position_publisher");
    this->posePub = this->rosNode->create_publisher<geometry_msgs::msg::PoseStamped>("abs_pose", 10);

    auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->model.Entity());
    auto const pose = poseComp->Data();

    X = pose.Pos().X();
    Y = pose.Pos().Y();
    Z = pose.Pos().Z();

    RotX = pose.Pos().X();
    RotY = pose.Pos().Y();
    RotZ = pose.Pos().Z();
    RotW = pose.Pos().W();

    std::cout << "PositionPublisher confirmed for entity [" << this->model.Name(_ecm) << "]" << std::endl;
}

void PositionPublisher::PostUpdate(gz::sim::UpdateInfo const &_info, gz::sim::EntityComponentManager const &_ecm)
{
    auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->model.Entity());
    auto const pose = poseComp->Data();

    geometry_msgs::msg::PoseStamped poseMsg;
    poseMsg.header.stamp = this->rosNode->get_clock()->now();
    poseMsg.header.frame_id = this->frameName_;
    poseMsg.pose.position.x = pose.Pos().X() - X;
    poseMsg.pose.position.y = pose.Pos().Y() - Y;
    poseMsg.pose.position.z = pose.Pos().Z() - Z;
    poseMsg.pose.orientation.x = pose.Rot().X() - RotX;
    poseMsg.pose.orientation.y = pose.Rot().Y() - RotY;
    poseMsg.pose.orientation.z = pose.Rot().Z() - RotZ;
    poseMsg.pose.orientation.w = pose.Rot().W() - RotW;

    this->posePub->publish(poseMsg);
}
}  // namespace position_publisher
