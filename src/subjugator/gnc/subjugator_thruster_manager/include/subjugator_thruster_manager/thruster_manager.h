#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

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
    Eigen::MatrixXd tam_;
    int const dof_ = 6;
    int const thruster_count_ = 8;
    double thruster_cap_;
    double max_force_pos_;
    double max_force_neg_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<subjugator_msgs::msg::ThrusterEfforts>::SharedPtr thrust_publisher_;

    void wrench_callback(geometry_msgs::msg::Wrench::SharedPtr msg);
    void timer_callback();
};