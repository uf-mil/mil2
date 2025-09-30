/*
 * Copyright (C) 2024 Rakesh Vivekanandan
 * Licensed under the Apache License, Version 2.0
 */

#include "DVLBridge.hh"

#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

/// Register the system with Gazebo Sim
GZ_ADD_PLUGIN(dave_ros_gz_plugins::DVLBridge, gz::sim::System, dave_ros_gz_plugins::DVLBridge::ISystemConfigure,
              dave_ros_gz_plugins::DVLBridge::ISystemPostUpdate)

namespace dave_ros_gz_plugins
{

// Scaling Covariance
double computeCovariance(double base_cov, double angular_speed_rad_s)
{
    double scale_factor = 1000.0;
    double scale = 10.0 + scale_factor * std::min(std::abs(angular_speed_rad_s), 2.0);
    return base_cov * scale;
}

struct DVLBridge::PrivateData
{
    std::mutex mutex;
    gz::transport::Node gz_node;

    std::string dvl_topic{ "/dvl/velocity" };

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dvl_pub;

    gz::sim::Entity baseEntity{ gz::sim::kNullEntity };

    gz::math::Pose3d lastBasePose{ 0, 0, 0, 0, 0, 0 };

    double currentYawRate = 0.0;
    std::chrono::steady_clock::time_point lastUpdateTime = std::chrono::steady_clock::now();
};

DVLBridge::DVLBridge() : dataPtr(std::make_unique<PrivateData>())
{
}

void DVLBridge::Configure(gz::sim::Entity const &_entity, std::shared_ptr<sdf::Element const> const &_sdf,
                          gz::sim::EntityComponentManager &, gz::sim::EventManager &)
{
    this->dataPtr->baseEntity = _entity;  // Remember the vehicle entity

    // Initialise rclcpp once
    if (!rclcpp::ok())
        rclcpp::init(0, nullptr);

    auto node = std::make_shared<rclcpp::Node>("dvl_bridge_node");
    this->ros_node_ = node;

    if (_sdf->HasElement("topic"))
        this->dataPtr->dvl_topic = _sdf->Get<std::string>("topic");

    gzmsg << "[DVLBridge] Gazebo DVL topic: " << this->dataPtr->dvl_topic << '\n';

    std::function<void(gz::msgs::DVLVelocityTracking const &)> cb =
        std::bind(&DVLBridge::receiveGazeboCallback, this, std::placeholders::_1);

    this->dataPtr->gz_node.Subscribe(this->dataPtr->dvl_topic, std::move(cb));
    this->dataPtr->dvl_pub = node->create_publisher<nav_msgs::msg::Odometry>("/dvl/odom", 10);
}

// Gazebo->ROS callback

void DVLBridge::receiveGazeboCallback(gz::msgs::DVLVelocityTracking const &msg)
{
    std::scoped_lock lk(this->dataPtr->mutex);

    gz::math::Vector3d v_body(msg.velocity().mean().x(), msg.velocity().mean().y(), msg.velocity().mean().z());

    nav_msgs::msg::Odometry odom;
    odom.header.stamp.sec = msg.header().stamp().sec();
    odom.header.stamp.nanosec = msg.header().stamp().nsec();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.twist.twist.linear.x = v_body.X();
    odom.twist.twist.linear.y = v_body.Y();
    odom.twist.twist.linear.z = v_body.Z();

    odom.pose.pose.orientation.w = 1.0;

    // Assigning Covariance
    constexpr double base_cov = 0.000001;
    double scaled_cov = computeCovariance(base_cov, this->dataPtr->currentYawRate);
    for (int k = 0; k < 3; ++k)
        odom.twist.covariance[k * 6 + k] = scaled_cov;

    this->dataPtr->dvl_pub->publish(odom);
}

void DVLBridge::PostUpdate(gz::sim::UpdateInfo const &_info, gz::sim::EntityComponentManager const &_ecm)
{
    if (_info.paused)
        return;

    if (auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->dataPtr->baseEntity))
    {
        gz::math::Pose3d currentPose = poseComp->Data();
        auto now = std::chrono::steady_clock::now();

        double dt = std::chrono::duration<double>(now - this->dataPtr->lastUpdateTime).count();

        if (dt > 0.0)
        {
            gz::math::Vector3d dAngles = (currentPose.Rot().Euler() - this->dataPtr->lastBasePose.Rot().Euler());
            this->dataPtr->currentYawRate = dAngles.Z() / dt;
        }

        this->dataPtr->lastUpdateTime = now;
        this->dataPtr->lastBasePose = currentPose;
    }

    rclcpp::spin_some(this->ros_node_);
}

}  // namespace dave_ros_gz_plugins
