/*
 * Copyright (C) 2024 Rakesh Vivekanandan
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "DVLBridge.hh"

#include <gz/common/Console.hh>
#include <gz/transport/Node.hh>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "gz/plugin/Register.hh"

GZ_ADD_PLUGIN(dave_ros_gz_plugins::DVLBridge, gz::sim::System, dave_ros_gz_plugins::DVLBridge::ISystemConfigure,
              dave_ros_gz_plugins::DVLBridge::ISystemPostUpdate)

namespace dave_ros_gz_plugins
{

struct DVLBridge::PrivateData
{
  // Add any private data members here.
  std::mutex mutex_;
  gz::transport::Node gz_node;
  std::string dvl_topic;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dvl_pub;
};

DVLBridge::DVLBridge() : dataPtr(std::make_unique<PrivateData>())
{
}

void DVLBridge::Configure(gz::sim::Entity const &_entity, std::shared_ptr<sdf::Element const> const &_sdf,
                          gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventManager)
{
  std::cout << "testing" << std::endl;
  gzdbg << "dave_ros_gz_plugins::DVLBridge::Configure on entity: " << _entity << std::endl;

  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  this->ros_node_ = std::make_shared<rclcpp::Node>("dvl_bridge_node");

  // Grab dvl topic from SDF
  if (!_sdf->HasElement("topic"))
  {
    this->dataPtr->dvl_topic = "/dvl/velocity";
    gzmsg << "dvl topic set to default:  " << this->dataPtr->dvl_topic << std::endl;
  }
  else
  {
    this->dataPtr->dvl_topic = _sdf->Get<std::string>("topic");
    gzmsg << "dvl topic: " << this->dataPtr->dvl_topic << std::endl;
  }

  // Gazebo subscriber
  std::function<void(gz::msgs::DVLVelocityTracking const &)> callback =
      std::bind(&DVLBridge::receiveGazeboCallback, this, std::placeholders::_1);

  this->dataPtr->gz_node.Subscribe(this->dataPtr->dvl_topic, callback);

  // ROS2 publisher
  this->dataPtr->dvl_pub = this->ros_node_->create_publisher<nav_msgs::msg::Odometry>(this->dataPtr->dvl_topic, 1);
}

void DVLBridge::receiveGazeboCallback(gz::msgs::DVLVelocityTracking const &msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex_);

  gzmsg << "dave_ros_gz_plugins::DVLBridge::receiveGazeboCallback" << std::endl;

  auto dvl_msg = nav_msgs::msg::Odometry();
  dvl_msg.header.stamp.sec = msg.header().stamp().sec();
  dvl_msg.header.stamp.nanosec = msg.header().stamp().nsec();
  dvl_msg.header.frame_id = "base_link";
  dvl_msg.child_frame_id = "dvl_sensor_link";

  dvl_msg.twist.twist.linear.x = msg.velocity().mean().x();
  dvl_msg.twist.twist.linear.y = msg.velocity().mean().y();
  dvl_msg.twist.twist.linear.z = msg.velocity().mean().z();

  // Simulated DVL does not produce a pose (at least not one we can use)
  dvl_msg.pose.pose.position.x = 0;
  dvl_msg.pose.pose.position.y = 0;
  dvl_msg.pose.pose.position.z = 0;

  // Limit to 9 so the output is not bloated with empty covariance values
  size_t covariance_size = msg.velocity().covariance().size();

  // Only filling upper-left most 9 spaces in 6x6 matrix
  for (size_t i = 0; i < 3; i++)
  {
    for (size_t j = 0; j < 3; j++)
    {
      dvl_msg.twist.covariance[i * 6 + j] = msg.velocity().covariance()[i * 3 + j];
    }
  }

  this->dataPtr->dvl_pub->publish(dvl_msg);
}

void DVLBridge::PostUpdate(gz::sim::UpdateInfo const &_info, gz::sim::EntityComponentManager const &_ecm)
{
  if (!_info.paused)
  {
    rclcpp::spin_some(this->ros_node_);

    if (_info.iterations % 1000 == 0)
    {
      gzmsg << "dave_ros_gz_plugins::DVLBridge::PostUpdate" << std::endl;
    }
  }
}

}  // namespace dave_ros_gz_plugins
