#ifndef ACTIVE_SONAR_HH_
#define ACTIVE_SONAR_HH_

#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <sensor_msgs/msg/laser_scan.hpp>


// NOTE: .msg imports must be snake_case even though the file is in PascalCase
#include "mil_msgs/msg/echo_intensities.hpp"

namespace active_sonar
{
  class ActiveSonar : 
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
  {
    public: 
      ActiveSonar();
      ~ActiveSonar();
 
      void Configure(
      const gz::sim::Entity & entity, 
      const std::shared_ptr<const sdf::Element> & sdf,
      gz::sim::EntityComponentManager & ecm,
      gz::sim::EventManager & eventMgr) override;

      void PostUpdate(const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) override;

      void receiveGazeboCallbackScan(const gz::msgs::LaserScan &msg);
      void receiveGazeboCallbackEcho(const gz::msgs::PointCloudPacked &msg);

    private: 
      std::shared_ptr<rclcpp::Node> ros_node_;
      
      struct PrivateData;
      std::unique_ptr<PrivateData> dataPtr;
  };
}

#endif