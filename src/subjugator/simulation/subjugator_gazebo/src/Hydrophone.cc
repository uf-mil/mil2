#include "../include/Hydrophone.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>  // For GZ_ADD_PLUGIN

namespace hydrophones
{

Hydrophone::Hydrophone()
{
}

Hydrophone::~Hydrophone()
{
}

void Hydrophone::Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                           gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr)
{
  this->modelEntity_ = entity;

  // Get pose values from SDF file
  if (sdf->HasElement("pose"))
    this->pose_ = sdf->Get<gz::math::Pose3d>("pose");

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  // this->rosNode_ = rclcpp::Node::make_shared("depth_sensor_node");
  // this->depthPub_ = this->rosNode_->create_publisher<mil_msgs::msg::DepthStamped>("/depth", 10);

  gzdbg << "[Hydrophone] Configure() done." << "\n"
        << "Entity = " << entity << "\n"
        << "Pose = " << this->pose_ << "\n";
}

void Hydrophone::PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm)
{
  // Convert simTime (steady_clock::duration) to float (seconds)
  float nowSec = std::chrono::duration_cast<std::chrono::duration<float>>(info.simTime).count();

  float dt = nowSec - this->lastPubTime_;
  if (dt < this->updatePeriod_)
    return;

  this->lastPubTime_ = nowSec;

  // Retrieve the Pose component
  auto poseComp = ecm.Component<gz::sim::components::Pose>(this->modelEntity_);
  if (!poseComp)
  {
    gzerr << "[DepthSensor] Pose component not found for entity [" << this->modelEntity_ << "]\n";
    return;
  }

  // To avoid the deprecated operator+, we manually combine:
  gz::math::Pose3d finalPose = poseComp->Data();
  finalPose.Pos() += this->offset_.Pos();
  finalPose.Rot() = finalPose.Rot() * this->offset_.Rot();

  // Depth is negative Z
  float depth = -finalPose.Pos().Z();
  if (depth < 0.0)
    depth = 0.0;

  // Publish
  mil_msgs::msg::DepthStamped msg;
  msg.header.frame_id = this->frameName_;
  msg.header.stamp = this->rosNode_->now();  // ROS time
  msg.depth = depth;

  this->depthPub_->publish(msg);
}

}  // namespace hydrophones

// Register plugin so Gazebo can see it
GZ_ADD_PLUGIN(hydrophone::Hydrophone, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
