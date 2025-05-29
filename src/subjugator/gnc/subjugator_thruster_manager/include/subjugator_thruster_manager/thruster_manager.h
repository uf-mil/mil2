#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/string.hpp"
#include "subjugator_msgs/msg/thruster_efforts.hpp"
class ThrusterManager : public rclcpp::Node
{
  public:
    ThrusterManager();

  private:
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_subscription_;
    Eigen::VectorXd reference_wrench_;  // TODO: change to Vector6d
    Eigen::Vector3d linear_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_vel_ = Eigen::Vector3d::Zero();
    Eigen::MatrixXd tam_;

    int const dof_ = 6;
    int const thruster_count_ = 8;
    double thruster_cap_;
    double max_force_pos_;
    double max_force_neg_;

    double k_surge_, k_sway_, k_heave_;
    double k_roll_, k_pitch_, k_yaw_;
    Eigen::Matrix3d k_asym_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<subjugator_msgs::msg::ThrusterEfforts>::SharedPtr thrust_publisher_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void wrench_callback(geometry_msgs::msg::Wrench::SharedPtr msg);
    void timer_callback();
};
