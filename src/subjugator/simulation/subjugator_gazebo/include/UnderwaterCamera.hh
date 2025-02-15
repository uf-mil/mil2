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

#ifndef DAVE_GZ_SENSOR_PLUGINS__UNDERWATERCAMERA_HH_
#define DAVE_GZ_SENSOR_PLUGINS__UNDERWATERCAMERA_HH_

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>

#include <gz/math/Angle.hh>
#include <gz/sim/System.hh>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace dave_gz_sensor_plugins

{
class UnderwaterCamera : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
public:
  UnderwaterCamera();
  ~UnderwaterCamera();

  void Configure(gz::sim::Entity const& entity, std::shared_ptr<sdf::Element const> const& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& eventMgr) override;

  void PostUpdate(gz::sim::UpdateInfo const& info, gz::sim::EntityComponentManager const& ecm) override;

  void CameraCallback(gz::msgs::Image const& image);

  void CameraInfoCallback(gz::msgs::CameraInfo const& cameraInfo);

  void DepthImageCallback(gz::msgs::Image const& image);

  cv::Mat ConvertGazeboToOpenCV(gz::msgs::Image const& gz_image);

  cv::Mat SimulateUnderwater(cv::Mat const& _inputImage, cv::Mat const& _inputDepth, cv::Mat& _outputImage);

private:
  std::shared_ptr<rclcpp::Node> ros_node_;

  struct PrivateData;
  std::unique_ptr<PrivateData> dataPtr;
};

std::string sanitizeNodeName(std::string const& name);

}  // namespace dave_gz_sensor_plugins

#endif  // DAVE_GZ_SENSOR_PLUGINS__UNDERWATERCAMERA_HH_
