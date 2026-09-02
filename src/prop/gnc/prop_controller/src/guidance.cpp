#include "prop_controller/guidance.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

namespace
{
double wrap(double angle)
{
    return std::remainder(angle, 2.0 * M_PI);
}

double yaw_of(geometry_msgs::msg::Quaternion const& q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

Guidance::Guidance() : Node("guidance")
{
    speed_ = declare_parameter("speed", 1.5);
    lookahead_ = declare_parameter("lookahead", 4.0);
    accept_radius_ = declare_parameter("accept_radius", 1.0);
    kp_heading_ = declare_parameter("kp_heading", 1.2);
    max_yaw_rate_ = declare_parameter("max_yaw_rate", 0.6);
    hold_radius_ = declare_parameter("hold_radius", 1.0);
    approach_gain_ = declare_parameter("approach_gain", 0.5);
    double const rate = declare_parameter("rate", 10.0);

    rclcpp::QoS latched(1);
    latched.transient_local();

    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
        "plan", latched, [this](nav_msgs::msg::Path::SharedPtr const msg) { plan_callback(*msg); });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry/filtered/global", 10,
                                                             [this](nav_msgs::msg::Odometry::SharedPtr const msg)
                                                             {
                                                                 position_ = { msg->pose.pose.position.x,
                                                                               msg->pose.pose.position.y };
                                                                 heading_ = yaw_of(msg->pose.pose.orientation);
                                                                 located_ = true;
                                                             });

    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { step(); });
}

void Guidance::plan_callback(nav_msgs::msg::Path const& path)
{
    waypoints_.clear();
    for (auto const& pose : path.poses)
    {
        waypoints_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    target_ = 0;
    leg_start_ = located_ ? position_ : Point{ 0.0, 0.0 };
    RCLCPP_INFO(get_logger(), "following %zu waypoints", waypoints_.size());
}

void Guidance::step()
{
    if (!located_ || waypoints_.empty())
    {
        return;
    }

    if (target_ < waypoints_.size() && std::hypot(position_.first - waypoints_[target_].first,
                                                  position_.second - waypoints_[target_].second) < accept_radius_)
    {
        leg_start_ = waypoints_[target_];
        if (++target_ == waypoints_.size())
        {
            RCLCPP_INFO(get_logger(), "holding station on the final waypoint");
        }
    }

    auto const [speed, error] = target_ < waypoints_.size() ? follow() : hold();

    geometry_msgs::msg::Twist command;
    command.linear.x = speed;
    command.angular.z = std::clamp(kp_heading_ * error, -max_yaw_rate_, max_yaw_rate_);
    command_pub_->publish(command);
}

std::pair<double, double> Guidance::follow() const
{
    // Touch up on trig
    Point const& goal = waypoints_[target_];
    double const bearing = std::atan2(goal.second - leg_start_.second, goal.first - leg_start_.first);
    double const cross = std::cos(bearing) * (position_.second - leg_start_.second) -
                         std::sin(bearing) * (position_.first - leg_start_.first);
    double const error = wrap(bearing + std::atan2(-cross, lookahead_) - heading_);

    double speed = speed_ * std::max(0.0, std::cos(error));
    if (target_ + 1 == waypoints_.size())
    {
        double const remaining = std::hypot(position_.first - goal.first, position_.second - goal.second);
        speed = std::min(speed, approach_gain_ * remaining);
    }
    return { speed, error };
}

std::pair<double, double> Guidance::hold() const
{
    Point const& goal = waypoints_.back();
    double const remaining = std::hypot(position_.first - goal.first, position_.second - goal.second);
    if (remaining < hold_radius_)
    {
        return { 0.0, 0.0 };
    }
    double const error = wrap(std::atan2(goal.second - position_.second, goal.first - position_.first) - heading_);
    return { approach_gain_ * remaining * std::max(0.0, std::cos(error)), error };
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Guidance>());
    rclcpp::shutdown();
    return 0;
}
