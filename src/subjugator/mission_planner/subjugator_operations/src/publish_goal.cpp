#include "publish_goal.hpp"

#include <geometry_msgs/msg/pose.hpp>

BT::PortsList PublishGoalPose::providedPorts()
{
    BT::PortsList ports;
    // Inputs
    ports.insert(BT::InputPort<double>("x"));
    ports.insert(BT::InputPort<double>("y"));
    ports.insert(BT::InputPort<double>("z"));
    ports.insert(BT::InputPort<double>("qx"));
    ports.insert(BT::InputPort<double>("qy"));
    ports.insert(BT::InputPort<double>("qz"));
    ports.insert(BT::InputPort<double>("qw"));
    ports.insert(BT::InputPort<bool>("relative", false, "Interpret goal relative to current pose"));

    // Outputs: resolved absolute goal (so other nodes can consume it)
    ports.insert(BT::OutputPort<double>("abs_x"));
    ports.insert(BT::OutputPort<double>("abs_y"));
    ports.insert(BT::OutputPort<double>("abs_z"));
    ports.insert(BT::OutputPort<double>("abs_qx"));
    ports.insert(BT::OutputPort<double>("abs_qy"));
    ports.insert(BT::OutputPort<double>("abs_qz"));
    ports.insert(BT::OutputPort<double>("abs_qw"));
    return ports;
}

static geometry_msgs::msg::Pose rotateVectorByQuat(geometry_msgs::msg::Pose const& ref, double rx, double ry, double rz)
{
    auto const& q = ref.orientation;
    double vx = rx, vy = ry, vz = rz;
    double qw = q.w, qx = q.x, qy = q.y, qz = q.z;

    double w1 = -qx * vx - qy * vy - qz * vz;
    double x1 = qw * vx + qy * vz - qz * vy;
    double y1 = qw * vy + qz * vx - qx * vz;
    double z1 = qw * vz + qx * vy - qy * vx;

    geometry_msgs::msg::Pose out = ref;
    out.position.x = x1 * qw - w1 * qx - y1 * qz + z1 * qy;
    out.position.y = y1 * qw - w1 * qy - z1 * qx + x1 * qz;
    out.position.z = z1 * qw - w1 * qz - x1 * qy + y1 * qx;
    return out;
}

geometry_msgs::msg::Pose PublishGoalPose::composeAbsoluteGoal_(geometry_msgs::msg::Pose const& current, double rx,
                                                               double ry, double rz, double qx, double qy, double qz,
                                                               double qw, bool relative)
{
    geometry_msgs::msg::Pose goal;
    if (relative)
    {
        auto rel_rotated = rotateVectorByQuat(current, rx, ry, rz);
        goal.position.x = current.position.x + rel_rotated.position.x;
        goal.position.y = current.position.y + rel_rotated.position.y;
        goal.position.z = current.position.z + rel_rotated.position.z;

        auto const& c = current.orientation;
        goal.orientation.x = c.w * qx + c.x * qw + c.y * qz - c.z * qy;
        goal.orientation.y = c.w * qy - c.x * qz + c.y * qw + c.z * qx;
        goal.orientation.z = c.w * qz + c.x * qy - c.y * qx + c.z * qw;
        goal.orientation.w = c.w * qw - c.x * qx - c.y * qy - c.z * qz;
    }
    else
    {
        goal.position.x = rx;
        goal.position.y = ry;
        goal.position.z = rz;
        goal.orientation.x = qx;
        goal.orientation.y = qy;
        goal.orientation.z = qz;
        goal.orientation.w = qw;
    }
    return goal;
}

BT::NodeStatus PublishGoalPose::tick()
{
    double x, y, z, qx, qy, qz, qw;
    bool relative = false;
    if (!getInput("x", x) || !getInput("y", y) || !getInput("z", z) || !getInput("qx", qx) || !getInput("qy", qy) ||
        !getInput("qz", qz) || !getInput("qw", qw) || !getInput("relative", relative))
    {
        RCLCPP_ERROR(ctx_->logger(), "PublishGoalPose: missing required inputs.");
        return BT::NodeStatus::FAILURE;
    }

    std::optional<nav_msgs::msg::Odometry> odom;
    {
        std::scoped_lock lk(ctx_->odom_mx);
        odom = ctx_->latest_odom;
    }
    geometry_msgs::msg::Pose current{};
    if (odom)
        current = odom->pose.pose;

    geometry_msgs::msg::Pose goal = composeAbsoluteGoal_(current, x, y, z, qx, qy, qz, qw, relative);

    // Publish
    ctx_->goal_pub->publish(goal);
    RCLCPP_INFO(ctx_->logger(),
                "PublishGoalPose: sent goal pos(%.2f,%.2f,%.2f) quat(%.2f,%.2f,%.2f,%.2f) [relative=%s]",
                goal.position.x, goal.position.y, goal.position.z, goal.orientation.x, goal.orientation.y,
                goal.orientation.z, goal.orientation.w, relative ? "true" : "false");

    // Expose absolute goal via outputs for downstream nodes (e.g., AtGoalPose)
    setOutput("abs_x", goal.position.x);
    setOutput("abs_y", goal.position.y);
    setOutput("abs_z", goal.position.z);
    setOutput("abs_qx", goal.orientation.x);
    setOutput("abs_qy", goal.orientation.y);
    setOutput("abs_qz", goal.orientation.z);
    setOutput("abs_qw", goal.orientation.w);

    return BT::NodeStatus::SUCCESS;
}
