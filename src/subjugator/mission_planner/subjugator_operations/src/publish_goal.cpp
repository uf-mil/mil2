#include "publish_goal.hpp"

#include <cmath>

#include <geometry_msgs/msg/pose.hpp>

BT::PortsList PublishGoalPose::providedPorts()
{
    BT::PortsList ports;

    // Defaults so yaw-only calls can omit them
    ports.insert(BT::InputPort<double>("x", 0.0, "Relative/absolute x (m)"));
    ports.insert(BT::InputPort<double>("y", 0.0, "Relative/absolute y (m)"));
    ports.insert(BT::InputPort<double>("z", 0.0, "Relative/absolute z (m)"));

    // Quaternion (legacy, optional)
    ports.insert(BT::InputPort<double>("qx", 0.0, "Quaternion x"));
    ports.insert(BT::InputPort<double>("qy", 0.0, "Quaternion y"));
    ports.insert(BT::InputPort<double>("qz", 0.0, "Quaternion z"));
    ports.insert(BT::InputPort<double>("qw", 1.0, "Quaternion w"));

    // Degrees API (default on)
    ports.insert(BT::InputPort<bool>("use_euler_deg", true, "Use roll/pitch/yaw degrees"));
    ports.insert(BT::InputPort<bool>("keep_current_pos_abs", true,
                                     "If absolute orientation, keep current position instead of 0,0,0"));

    ports.insert(BT::InputPort<double>("roll_deg", 0.0, "Roll (deg)"));
    ports.insert(BT::InputPort<double>("pitch_deg", 0.0, "Pitch (deg)"));
    ports.insert(BT::InputPort<double>("yaw_deg", 0.0, "Yaw (deg)"));

    // Chain relative by default
    ports.insert(BT::InputPort<bool>("relative", true, "Relative to last goal"));

    // Context + outputs
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));
    ports.insert(BT::OutputPort<double>("abs_x"));
    ports.insert(BT::OutputPort<double>("abs_y"));
    ports.insert(BT::OutputPort<double>("abs_z"));
    ports.insert(BT::OutputPort<double>("abs_qx"));
    ports.insert(BT::OutputPort<double>("abs_qy"));
    ports.insert(BT::OutputPort<double>("abs_qz"));
    ports.insert(BT::OutputPort<double>("abs_qw"));

    return ports;
}

static inline double deg2rad(double d)
{
    return d * M_PI / 180.0;
}

void PublishGoalPose::quatFromDeg(double r_deg, double p_deg, double y_deg, double& qx, double& qy, double& qz,
                                  double& qw)
{
    double cr = std::cos(deg2rad(r_deg) * 0.5);
    double sr = std::sin(deg2rad(r_deg) * 0.5);
    double cp = std::cos(deg2rad(p_deg) * 0.5);
    double sp = std::sin(deg2rad(p_deg) * 0.5);
    double cy = std::cos(deg2rad(y_deg) * 0.5);
    double sy = std::sin(deg2rad(y_deg) * 0.5);

    // ZYX convention: q = qz(yaw)*qy(pitch)*qx(roll)
    qw = cy * cp * cr + sy * sp * sr;
    qx = cy * cp * sr - sy * sp * cr;
    qy = cy * sp * cr + sy * cp * sr;
    qz = sy * cp * cr - cy * sp * sr;
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
    if (!ctx_)
    {
        if (!getInput("ctx", ctx_) || !ctx_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "PublishGoalPose: missing ctx on blackboard");
            return BT::NodeStatus::FAILURE;
        }
    }

    // Defaults (match providedPorts)
    double x = 0.0, y = 0.0, z = 0.0;
    bool relative = true;
    (void)getInput("x", x);
    (void)getInput("y", y);
    (void)getInput("z", z);
    (void)getInput("relative", relative);

    // Orientation: degrees by default; fallback to quaternion if requested
    bool use_euler = true;
    (void)getInput("use_euler_deg", use_euler);

    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;

    if (use_euler)
    {
        (void)getInput("roll_deg", roll_deg);
        (void)getInput("pitch_deg", pitch_deg);
        (void)getInput("yaw_deg", yaw_deg);
        quatFromDeg(roll_deg, pitch_deg, yaw_deg, qx, qy, qz, qw);
    }
    else
    {
        if (!getInput("qx", qx) || !getInput("qy", qy) || !getInput("qz", qz) || !getInput("qw", qw))
        {
            RCLCPP_ERROR(ctx_->logger(), "PublishGoalPose: missing quaternion inputs and use_euler_deg==false.");
            return BT::NodeStatus::FAILURE;
        }
    }

    // Optional: nudge forward a hair for yaw-only steps so low-level controllers act
    auto clamp = [](double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); };
    double angle_rad = 2.0 * std::acos(clamp(qw, -1.0, 1.0));
    double angle_deg = angle_rad * 180.0 / M_PI;
    double const eps_move = 0.002;  // 2 mm
    if (relative && std::fabs(x) < 1e-6 && std::fabs(y) < 1e-6 && std::fabs(z) < 1e-6 && angle_deg > 0.5)
    {
        x = eps_move;
        RCLCPP_DEBUG(ctx_->logger(), "PublishGoalPose: yaw-only -> adding %.3fm forward nudge", eps_move);
    }

    // Choose base pose (last goal if available when relative, else odom, else identity)
    geometry_msgs::msg::Pose base{};
    bool have_base = false;

    if (relative)
    {
        std::scoped_lock lk(ctx_->last_goal_mx);
        if (ctx_->last_goal)
        {
            base = *ctx_->last_goal;
            have_base = true;
        }
    }
    if (!have_base)
    {
        std::optional<nav_msgs::msg::Odometry> odom;
        {
            std::scoped_lock lk(ctx_->odom_mx);
            odom = ctx_->latest_odom;
        }
        if (odom)
        {
            base = odom->pose.pose;
            have_base = true;
        }
    }
    if (!have_base)
    {
        base.position.x = base.position.y = base.position.z = 0.0;
        base.orientation.x = base.orientation.y = base.orientation.z = 0.0;
        base.orientation.w = 1.0;
        RCLCPP_WARN(ctx_->logger(), "PublishGoalPose: composing without odom; assuming world origin.");
    }

    bool keep_cur_abs = true;
    (void)getInput("keep_current_pos_abs", keep_cur_abs);
    if (!relative && keep_cur_abs)
    {
        x = base.position.x;
        y = base.position.y;
        z = base.position.z;
    }

    // Compose absolute goal and publish
    geometry_msgs::msg::Pose goal = composeAbsoluteGoal_(base, x, y, z, qx, qy, qz, qw, relative);
    ctx_->goal_pub->publish(goal);

    // Remember as last goal for chaining
    {
        std::scoped_lock lk(ctx_->last_goal_mx);
        ctx_->last_goal = goal;
    }

    RCLCPP_INFO(ctx_->logger(),
                "PublishGoalPose: in(rel=%d x=%.3f y=%.3f z=%.3f yaw_deg=%.1f use_euler=%d) -> "
                "sent pos(%.2f,%.2f,%.2f) quat(%.3f,%.3f,%.3f,%.3f) [base=%s]",
                (int)relative, x, y, z, yaw_deg, (int)use_euler, goal.position.x, goal.position.y, goal.position.z,
                goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w,
                have_base ? "set" : "none");

    // Outputs (absolute goal)
    setOutput("abs_x", goal.position.x);
    setOutput("abs_y", goal.position.y);
    setOutput("abs_z", goal.position.z);
    setOutput("abs_qx", goal.orientation.x);
    setOutput("abs_qy", goal.orientation.y);
    setOutput("abs_qz", goal.orientation.z);
    setOutput("abs_qw", goal.orientation.w);

    return BT::NodeStatus::SUCCESS;
}
