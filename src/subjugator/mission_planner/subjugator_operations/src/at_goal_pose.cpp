#include "at_goal_pose.hpp"

#include <cmath>

BT::PortsList AtGoalPose::providedPorts()
{
    BT::PortsList ports;
    ports.insert(BT::InputPort<double>("x"));
    ports.insert(BT::InputPort<double>("y"));
    ports.insert(BT::InputPort<double>("z"));
    ports.insert(BT::InputPort<double>("qx"));
    ports.insert(BT::InputPort<double>("qy"));
    ports.insert(BT::InputPort<double>("qz"));
    ports.insert(BT::InputPort<double>("qw"));
    ports.insert(BT::InputPort<double>("pos_tol", 0.20, "Position tolerance (m)"));
    ports.insert(BT::InputPort<double>("ori_tol_deg", 10.0, "Orientation tolerance (deg)"));
    return ports;
}

double AtGoalPose::quatAngularErrorDeg_(double qx, double qy, double qz, double qw, double rx, double ry, double rz,
                                        double rw)
{
    double const dot = std::fabs(qx * rx + qy * ry + qz * rz + qw * rw);
    double const clamped = std::min(1.0, std::max(0.0, dot));
    double angle = 2.0 * std::acos(clamped);  // radians
    return angle * 180.0 / M_PI;
}

BT::NodeStatus AtGoalPose::tick()
{
    double gx, gy, gz, gqx, gqy, gqz, gqw, pos_tol, ori_tol_deg;
    if (!getInput("x", gx) || !getInput("y", gy) || !getInput("z", gz) || !getInput("qx", gqx) ||
        !getInput("qy", gqy) || !getInput("qz", gqz) || !getInput("qw", gqw) || !getInput("pos_tol", pos_tol) ||
        !getInput("ori_tol_deg", ori_tol_deg))
    {
        RCLCPP_ERROR(ctx_->logger(), "AtGoalPose: missing required inputs.");
        return BT::NodeStatus::FAILURE;
    }

    std::optional<nav_msgs::msg::Odometry> odom;
    {
        std::scoped_lock lk(ctx_->odom_mx);
        odom = ctx_->latest_odom;
    }
    if (!odom)
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "AtGoalPose: no odometry yet.");
        return BT::NodeStatus::FAILURE;
    }

    auto const& p = odom->pose.pose.position;
    auto const& q = odom->pose.pose.orientation;

    double const dist = std::sqrt((p.x - gx) * (p.x - gx) + (p.y - gy) * (p.y - gy) + (p.z - gz) * (p.z - gz));
    double const ang = quatAngularErrorDeg_(q.x, q.y, q.z, q.w, gqx, gqy, gqz, gqw);

    bool const ok = (dist <= pos_tol) && (ang <= ori_tol_deg);

    RCLCPP_DEBUG_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 500, "AtGoalPose: dist=%.2f(m) ang=%.1f(deg) -> %s",
                          dist, ang, ok ? "SUCCESS" : "FAIL");

    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
