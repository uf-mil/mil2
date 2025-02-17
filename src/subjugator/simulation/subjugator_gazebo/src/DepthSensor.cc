#include "DepthSensor.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>  // For GZ_ADD_PLUGIN


namespace depth_sensor
{
  
DepthSensor::DepthSensor()
{
}

DepthSensor::~DepthSensor()
{
}

void DepthSensor::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf, gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr)
{
  this->modelEntity_ = entity;

  // Frame from SDF
  if (sdf->HasElement("frame_id"))
    this->frameName_ = sdf->Get<std::string>("frame_id");

  // Offset from SDF
  if (sdf->HasElement("offset"))
    this->offset_ = sdf->Get<gz::math::Pose3d>("offset");

  // Update rate from SDF -> update period
  if (sdf->HasElement("update_rate"))
  {
    float rate = sdf->Get<float>("update_rate");
    if (rate > 0.0)
      this->updatePeriod_ = 1.0 / rate;
  }

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  this->rosNode_  = rclcpp::Node::make_shared("depth_sensor_node");
  this->depthPub_ = this->rosNode_->create_publisher<mil_msgs::msg::DepthStamped>("/depth", 10);

  gzdbg << "[DepthSensor] Configure() done. Entity=" << entity
        << " frame_id=" << this->frameName_ 
        << " update_period=" << this->updatePeriod_ << "s\n";
}

void DepthSensor::PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm)
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
    gzerr << "[DepthSensor] Pose component not found for entity [" 
          << this->modelEntity_ << "]\n";
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
  msg.header.stamp    = this->rosNode_->now(); // ROS time
  msg.depth           = depth;

  this->depthPub_->publish(msg);
}

} 

// Register plugin so Gazebo can see it
GZ_ADD_PLUGIN(depth_sensor::DepthSensor,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPostUpdate)
