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
 *
 */

#ifndef DAVE_ROS_GZ_PLUGINS__DVLBRIDGE_HH_
#define DAVE_ROS_GZ_PLUGINS__DVLBRIDGE_HH_

#include <gz/msgs/dvl_velocity_tracking.pb.h>

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <gz/sim/System.hh>
#include <nav_msgs/msg/odometry.hpp>

namespace dave_ros_gz_plugins

{
class DVLBridge : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
  public:
    DVLBridge();
    ~DVLBridge() override = default;

    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

    void PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm) override;

    void receiveGazeboCallback(gz::msgs::DVLVelocityTracking const &dvl_velocity_tracking);

  private:
    std::shared_ptr<rclcpp::Node> ros_node_;

    struct PrivateData;
    std::unique_ptr<PrivateData> dataPtr;
};
}  // namespace dave_ros_gz_plugins

#endif  // DAVE_ROS_GZ_PLUGINS__DVLBRIDGE_HH_
