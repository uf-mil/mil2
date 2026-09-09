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
    yaw_tolerance_ = declare_parameter("yaw_tolerance", 0.09);
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
    holding_heading_ = false;

    // Only the last pose's orientation means anything: the ones in between are
    // driven through rather than stopped on. A quaternion of no length is not a
    // rotation, so it reads as "finish on any heading".
    has_goal_heading_ = false;
    if (!path.poses.empty())
    {
        auto const& q = path.poses.back().pose.orientation;
        if (std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 1e-6)
        {
            has_goal_heading_ = true;
            goal_heading_ = yaw_of(q);
        }
    }

    if (has_goal_heading_)
    {
        RCLCPP_INFO(get_logger(), "following %zu waypoints, finishing on %.0f degrees", waypoints_.size(),
                    goal_heading_ * 180.0 / M_PI);
    }
    else
    {
        RCLCPP_INFO(get_logger(), "following %zu waypoints, finishing on any heading", waypoints_.size());
    }
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

std::pair<double, double> Guidance::hold()
{
    Point const& goal = waypoints_.back();
    double const remaining = std::hypot(position_.first - goal.first, position_.second - goal.second);

    // Latched, and with room to drift before it lets go: turning on the spot
    // pushes the hull around, and on a bare threshold that hands control
    // straight back to the bearing law, which turns it back. The two then take
    // turns and the boat sits there wagging.
    if (remaining < hold_radius_)
    {
        holding_heading_ = true;
    }
    else if (remaining > 2.0 * hold_radius_)
    {
        holding_heading_ = false;
    }

    if (!holding_heading_)
    {
        // The stern reaches the point as well as the bow does. Folding the
        // error into a quarter turn either way aims whichever end is nearer,
        // and the projection onto the bow is then signed: negative when the
        // goal is behind, which the allocator spends as reverse thrust. That
        // is what backs it out of an overshoot. Steering the bow round instead
        // only works if the turning circle fits inside hold_radius, and at
        // these speeds it does not, so the boat circles the point forever.
        //
        // The fold flips the yaw command sign as the goal crosses abeam. A
        // dead band on the choice was measured and made no difference: hold
        // only runs near the goal, where the speed this asks for is small
        // enough that the flip costs nothing.
        double const to_goal =
            wrap(std::atan2(goal.second - position_.second, goal.first - position_.first) - heading_);
        return { approach_gain_ * remaining * std::cos(to_goal), std::remainder(to_goal, M_PI) };
    }

    // On the point, where the bearing to it is noise. Turn to the heading the
    // plan asked for, or sit still if it did not ask for one.
    if (!has_goal_heading_)
    {
        return { 0.0, 0.0 };
    }

    double const error = wrap(goal_heading_ - heading_);
    return { 0.0, std::abs(error) < yaw_tolerance_ ? 0.0 : error };
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Guidance>());
    rclcpp::shutdown();
    return 0;
}
