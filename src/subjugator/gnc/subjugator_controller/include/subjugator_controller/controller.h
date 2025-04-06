#pragma once

#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <control_toolbox/pid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>

class PIDController : public rclcpp::Node
{
  public:
    PIDController();
    void odom_cb(nav_msgs::msg::Odometry::UniquePtr const msg);
    void goal_trajectory_cb(geometry_msgs::msg::Pose::UniquePtr const msg);
    void publish_commands(std::array<double, 6> const &commands);
    void shutdown();

  private:
    bool is_shutdown;
    static int const dof_ = 6;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_goal_trajectory_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_cmd_wrench_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::unordered_map<std::string, std::pair<std::vector<double>, std::shared_ptr<rclcpp::ParameterCallbackHandle>>>
        param_map_;

    Eigen::Matrix<double, 7, 1> last_odom_;
    Eigen::Matrix<double, 7, 1> last_goal_trajectory_;
    std::array<control_toolbox::Pid, 6> pid_vec_;
    void control_loop();
    rclcpp::Time last_cmd_time_;
};
