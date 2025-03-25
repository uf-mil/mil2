#pragma once

#include <Eigen/Core>
#include <array>
#include <control_toolbox/pid.hpp>
#include <iostream>
#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class PIDController : public rclcpp::Node
{
  public:
    PIDController();
    void odom_cb(nav_msgs::msg::Odometry::UniquePtr const msg);
    void goal_trajectory_cb(geometry_msgs::msg::Pose::UniquePtr const msg);
    void publish_commands(std::array<double, 6> const &commands);

  private:
    static int const dof_ = 6;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_goal_trajectory_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_cmd_wrench_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> kp_cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> ki_cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> kd_cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> imax_cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> imin_cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> antiwindup_cb_handle_;

    Eigen::Matrix<double, 7, 1> last_odom_;  // Eigen::Vector7d won't work? typedefs are missing
    Eigen::Matrix<double, 7, 1> last_goal_trajectory_;
    std::array<control_toolbox::Pid, 6> pid_vec_;
    void control_loop();
    rclcpp::Time last_cmd_time_;
};
