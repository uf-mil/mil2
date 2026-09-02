#include "prop_controller/thruster_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

namespace
{
double clamp(double value, double limit)
{
    return std::clamp(value, -limit, limit);
}
}  // namespace

ThrusterManager::ThrusterManager() : Node("thruster_manager")
{
    rate_ = declare_parameter("rate", 20.0);
    command_timeout_ = declare_parameter("command_timeout", 1.0);
    max_thrust_ = declare_parameter("max_thrust", 45.0);
    thruster_y_ = declare_parameter("thruster_y", 0.25);
    kp_surge_ = declare_parameter("kp_surge", 60.0);
    ki_surge_ = declare_parameter("ki_surge", 30.0);
    kp_yaw_ = declare_parameter("kp_yaw", 30.0);
    ki_yaw_ = declare_parameter("ki_yaw", 15.0);

    surge_limit_ = 2 * max_thrust_;
    yaw_limit_ = 2 * max_thrust_ * thruster_y_;

    command_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
                                                                  [this](geometry_msgs::msg::Twist::SharedPtr const msg)
                                                                  {
                                                                      command_ = *msg;
                                                                      stamp_ = now().seconds();
                                                                  });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry/filtered/global", 10,
                                                             [this](nav_msgs::msg::Odometry::SharedPtr const msg)
                                                             {
                                                                 surge_ = msg->twist.twist.linear.x;
                                                                 yaw_rate_ = msg->twist.twist.angular.z;
                                                             });

    left_pub_ = create_publisher<std_msgs::msg::Float32>("thrusters/left", 10);
    right_pub_ = create_publisher<std_msgs::msg::Float32>("thrusters/right", 10);
    beat_pub_ = create_publisher<std_msgs::msg::Empty>("thrusters/heartbeat", 10);

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_), [this] { step(); });
}

void ThrusterManager::step()
{
    if (now().seconds() - stamp_ > command_timeout_)
    {
        surge_integral_ = 0.0;
        yaw_integral_ = 0.0;
        publish(0.0, 0.0);
        return;
    }

    double const dt = 1.0 / rate_;
    double const surge_error = command_.linear.x - surge_;
    double const yaw_error = command_.angular.z - yaw_rate_;
    surge_integral_ = clamp(surge_integral_ + surge_error * dt, surge_limit_ / ki_surge_);
    yaw_integral_ = clamp(yaw_integral_ + yaw_error * dt, yaw_limit_ / ki_yaw_);

    double const force = kp_surge_ * surge_error + ki_surge_ * surge_integral_;
    double const moment = kp_yaw_ * yaw_error + ki_yaw_ * yaw_integral_;

    // Both efforts are normalized against one thruster at full thrust, so the
    // turn takes what it needs and surge gets whatever headroom is left.
    double const turn = clamp(moment / thruster_y_ / 2 / max_thrust_, 1.0);
    double const surge = clamp(force / 2 / max_thrust_, 1.0 - std::abs(turn));
    publish(surge - turn, surge + turn);
    beat_pub_->publish(std_msgs::msg::Empty());
}

void ThrusterManager::publish(double left, double right)
{
    std_msgs::msg::Float32 message;
    message.data = static_cast<float>(left);
    left_pub_->publish(message);
    message.data = static_cast<float>(right);
    right_pub_->publish(message);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterManager>());
    rclcpp::shutdown();
    return 0;
}
