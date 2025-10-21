#include "publish_goal.hpp"

#include <geometry_msgs/msg/pose.hpp>

BT::PortsList PublishGoalPose::providedPorts()
{
    BT::PortsList ports;
    // Inputs
    ports.insert(BT::InputPort<double>("x"));
    ports.insert(BT::InputPort<double>("y"));
    ports.insert(BT::InputPort<double>("z"));
    ports.insert(BT::InputPort<double>("q.x"));
    ports.insert(BT::InputPort<double>("q.y"));
    ports.insert(BT::InputPort<double>("q.z"));
    ports.insert(BT::InputPort<double>("q.w"));
    ports.insert(BT::InputPort<bool>("relative", false, "Interpret goal relative to current pose"));
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx"));

    // Outputs: resolved absolute goal
    ports.insert(BT::OutputPort<double>("abs_x"));
    ports.insert(BT::OutputPort<double>("abs_y"));
    ports.insert(BT::OutputPort<double>("abs_z"));
    ports.insert(BT::OutputPort<double>("abs_qx"));
    ports.insert(BT::OutputPort<double>("abs_q.y"));
    ports.insert(BT::OutputPort<double>("abs_q.z"));
    ports.insert(BT::OutputPort<double>("abs_q.w"));

    return ports;
}

static geometry_msgs::msg::Pose rotateVectorByQuat(geometry_msgs::msg::Pose const& ref, double rx, double ry, double rz)
{
    auto const& q = ref.orientation;

    double w1 = -q.x * rx - q.y * ry - q.z * rz;
    double x1 = q.w * rx + q.y * rz - q.z * ry;
    double y1 = q.w * ry + q.z * rx - q.x * rz;
    double z1 = q.w * rz + q.x * ry - q.y * rx;

    geometry_msgs::msg::Pose out = ref;
    out.position.x = x1 * q.w - w1 * q.x - y1 * q.z + z1 * q.y;
    out.position.y = y1 * q.w - w1 * q.y - z1 * q.x + x1 * q.z;
    out.position.z = z1 * q.w - w1 * q.z - x1 * q.y + y1 * q.x;
    return out;
}

geometry_msgs::msg::Pose PublishGoalPose::composeAbsoluteGoal_(geometry_msgs::msg::Pose const& current, double rx,
                                                               double ry, double rz, double q.x, double q.y, double q.z,
                                                               double q.w, bool relative)
{
    geometry_msgs::msg::Pose goal;
    if (relative)
    {
        auto rel_rotated = rotateVectorByQuat(current, rx, ry, rz);
        goal.position.x = current.position.x + rel_rotated.position.x;
        goal.position.y = current.position.y + rel_rotated.position.y;
        goal.position.z = current.position.z + rel_rotated.position.z;

        auto const& c = current.orientation;
        goal.orientation.x = c.w * q.x + c.x * q.w + c.y * q.z - c.z * q.y;
        goal.orientation.y = c.w * q.y - c.x * q.z + c.y * q.w + c.z * q.x;
        goal.orientation.z = c.w * q.z + c.x * q.y - c.y * q.x + c.z * q.w;
        goal.orientation.w = c.w * q.w - c.x * q.x - c.y * q.y - c.z * q.z;
    }
    else
    {
        goal.position.x = rx;
        goal.position.y = ry;
        goal.position.z = rz;
        goal.orientation.x = q.x;
        goal.orientation.y = q.y;
        goal.orientation.z = q.z;
        goal.orientation.w = q.w;
    }
    return goal;
}

BT::NodeStatus PublishGoalPose::tick()
{
    if (!ctx_)
    {
        if (!getInput("ctx", ctx_) || !ctx_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "PublishGoalPose: missing ctx on blackboard");
            return BT::NodeStatus::FAILURE;
        }
    }

    double x, y, z, q.x, q.y, q.z, q.w;
    bool relative = false;

    if (!getInput("x", x) || !getInput("y", y) || !getInput("z", z) || !getInput("q.x", q.x) || !getInput("q.y", q.y) ||
        !getInput("q.z", q.z) || !getInput("q.w", q.w) || !getInput("relative", relative))
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

    geometry_msgs::msg::Pose goal = composeAbsoluteGoal_(current, x, y, z, q.x, q.y, q.z, q.w, relative);

    // Publish
    ctx_->goal_pub->publish(goal);
    RCLCPP_INFO(ctx_->logger(),
                "PublishGoalPose: sent goal pos(%.2f,%.2f,%.2f) quat(%.2f,%.2f,%.2f,%.2f) [relative=%s]",
                goal.position.x, goal.position.y, goal.position.z, goal.orientation.x, goal.orientation.y,
                goal.orientation.z, goal.orientation.w, relative ? "true" : "false");

    // Expose absolute goal via outputs
    setOutput("abs_x", goal.position.x);
    setOutput("abs_y", goal.position.y);
    setOutput("abs_z", goal.position.z);
    setOutput("abs_qx", goal.orientation.x);
    setOutput("abs_q.y", goal.orientation.y);
    setOutput("abs_q.z", goal.orientation.z);
    setOutput("abs_q.w", goal.orientation.w);

    return BT::NodeStatus::SUCCESS;
}
