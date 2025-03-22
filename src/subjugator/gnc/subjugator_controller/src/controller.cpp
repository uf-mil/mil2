#include "subjugator_controller/controller.h"

PIDController::PIDController() : Node("pid_controller")
{
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("subjugator_localization/odometry/filtered", 10,
                                                                   [this](nav_msgs::msg::Odometry::UniquePtr msg)
                                                                   { this->odom_cb(std::move(msg)); });
    sub_goal_trajectory_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "trajectory", 10,
        [this](geometry_msgs::msg::Pose::UniquePtr msg) { this->goal_trajectory_cb(std::move(msg)); });

    last_cmd_time_ = this->get_clock()->now();

    pid_ = control_toolbox::Pid(1.0, 0.0, 0.5, 0.0, 0.0, -0.0);

    control_loop();
}

void PIDController::control_loop()
{
    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        RCLCPP_INFO(this->get_logger(), "control loop");

        rclcpp::Time tnow = this->get_clock()->now();
        double dt_s = (tnow - last_cmd_time_).seconds();

        double error = last_goal_trajectory_.position.x - last_odom_.pose.pose.position.x;
        RCLCPP_INFO(this->get_logger(), "error: '%s'", std::to_string(error).c_str());
        double command = pid_.compute_command(error, dt_s);

        // this->command_system(command);
        RCLCPP_INFO(this->get_logger(), "command: '%s'", std::to_string(command).c_str());

        last_cmd_time_ = tnow;
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }
}

void PIDController::odom_cb(nav_msgs::msg::Odometry::UniquePtr const msg)
{
    last_odom_ = *msg;
    RCLCPP_INFO(this->get_logger(), "heard odom: '%s'", std::to_string(last_odom_.pose.pose.position.x).c_str());
}

void PIDController::goal_trajectory_cb(geometry_msgs::msg::Pose::UniquePtr const msg)
{
    last_goal_trajectory_ = *msg;
    RCLCPP_INFO(this->get_logger(), "heard goal: '%s'", std::to_string(last_goal_trajectory_.position.x).c_str());
}
