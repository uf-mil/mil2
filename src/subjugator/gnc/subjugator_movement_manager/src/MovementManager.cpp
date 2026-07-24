#include "subjugator_movement_manager/MovementManager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include <Eigen/Dense>

#include "mil_tools/geometry/Rotation.hpp"

using namespace std::chrono_literals;

MovementManager::MovementManager() : Node("movement_manager")
{
    pos_tol = declare_parameter("pos_tol", 0.2);
    ori_tol = declare_parameter("ori_tol_deg", 12.0);

    loose_pos_tol = declare_parameter("loose_pos_tol", 0.85);
    loose_ori_tol = declare_parameter("loose_ori_tol_deg", 40.0);

    precise_pos_tol = declare_parameter("precise_pos_tol", 0.08);
    precise_ori_tol = declare_parameter("precise_ori_tol_deg", 4.0);

    grace_secs = declare_parameter("grace_secs", 5.0);

    absolute_move_sub_ = create_subscription<subjugator_msgs::msg::AbsoluteMove>(
        "absolute_move", 10, [this](subjugator_msgs::msg::AbsoluteMove::SharedPtr msg) { absolute_move_cb(msg); });
    relative_move_sub_ = create_subscription<subjugator_msgs::msg::RelativeMove>(
        "relative_move", 10, [this](subjugator_msgs::msg::RelativeMove::SharedPtr msg) { relative_move_cb(msg); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_cb(msg); });

    goal_pub_ = create_publisher<geometry_msgs::msg::Pose>("goal_pose", 10);
    status_pub_ = create_publisher<subjugator_msgs::msg::MoveStatus>("move_status", 10);

    // Telemetry cadence: fast enough to watch a move settle, cheap enough to bag.
    status_timer_ = create_wall_timer(100ms, [this]() { publish_status_(); });
}

void MovementManager::odom_cb(nav_msgs::msg::Odometry::SharedPtr const &msg)
{
    latest_odom_ = *msg;
}

geometry_msgs::msg::Pose MovementManager::base_pose_(MoveRequest const &req) const
{
    if (req.relative && last_goal_)
    {
        return *last_goal_;
    }
    if (latest_odom_)
    {
        return latest_odom_->pose.pose;
    }

    RCLCPP_WARN(get_logger(), "MovementManager: no base pose available; assuming world origin.");
    geometry_msgs::msg::Pose origin;
    origin.orientation.w = 1.0;
    return origin;
}

geometry_msgs::msg::Pose MovementManager::resolve_goal_(MoveRequest const &req) const
{
    geometry_msgs::msg::Pose const base = base_pose_(req);

    if (req.relative)
    {
        // Rotate the body-frame offset into the world by the base orientation, then
        // stack the requested rotation on top of the base's.
        mil::geometry::Rotation const base_rot{ base.orientation };
        Eigen::Vector3d const offset =
            base_rot.rot_mat() * Eigen::Vector3d{ req.target.position.x, req.target.position.y, req.target.position.z };

        geometry_msgs::msg::Pose goal;
        goal.position.x = base.position.x + offset.x();
        goal.position.y = base.position.y + offset.y();
        goal.position.z = base.position.z + offset.z();
        goal.orientation = (base_rot * mil::geometry::Rotation{ req.target.orientation }).quat_msg();
        return goal;
    }

    // Absolute: start from the target, then hold current axes if asked.
    using subjugator_msgs::msg::AbsoluteMove;
    geometry_msgs::msg::Pose goal = req.target;
    if (req.hold & AbsoluteMove::HOLD_X)
    {
        goal.position.x = base.position.x;
    }
    if (req.hold & AbsoluteMove::HOLD_Y)
    {
        goal.position.y = base.position.y;
    }
    if (req.hold & AbsoluteMove::HOLD_Z)
    {
        goal.position.z = base.position.z;
    }
    if (req.hold & AbsoluteMove::HOLD_ORIENTATION)
    {
        goal.orientation = base.orientation;
    }
    return goal;
}

bool MovementManager::current_errors_(geometry_msgs::msg::Pose const &goal, double &dist, double &ang) const
{
    if (!latest_odom_)
    {
        return false;
    }
    auto const &pose = latest_odom_->pose.pose;
    double const dx = pose.position.x - goal.position.x;
    double const dy = pose.position.y - goal.position.y;
    double const dz = pose.position.z - goal.position.z;
    dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    ang = mil::geometry::Rotation{ pose.orientation }.quat().angularDistance(
              mil::geometry::Rotation{ goal.orientation }.quat()) *
          180.0 / M_PI;
    return true;
}

void MovementManager::emit_status_(uint8_t state, std::string const &message, double dist, double ang)
{
    subjugator_msgs::msg::MoveStatus status;
    status.header.stamp = now();
    status.header.frame_id = "odom";
    status.command_id = active_command_id_;
    status.goal = active_goal_;
    status.dist_remaining = dist;
    status.ori_err_deg = ang;
    status.at_goal = dist <= active_tight_pos_ && ang <= active_tight_ori_;
    status.state = state;
    status.message = message;
    status_pub_->publish(status);
}

void MovementManager::reject_no_odom_(uint32_t command_id)
{
    subjugator_msgs::msg::MoveStatus status;
    status.header.stamp = now();
    status.header.frame_id = "odom";
    status.command_id = command_id;
    status.state = subjugator_msgs::msg::MoveStatus::PREEMPTED;
    status.message = "No odometry received";
    status_pub_->publish(status);
    RCLCPP_WARN(get_logger(), "Move #%u rejected: no odometry received", command_id);
}

void MovementManager::absolute_move_cb(subjugator_msgs::msg::AbsoluteMove::SharedPtr const &msg)
{
    if (!latest_odom_)
    {
        reject_no_odom_(msg->command_id);
        return;
    }
    MoveRequest req;
    req.relative = false;
    req.target = msg->target;
    req.hold = msg->hold;
    req.precise = msg->precise;
    req.command_id = msg->command_id;
    drive_to_(req);
}

void MovementManager::relative_move_cb(subjugator_msgs::msg::RelativeMove::SharedPtr const &msg)
{
    if (!latest_odom_)
    {
        reject_no_odom_(msg->command_id);
        return;
    }
    // Convert the ergonomic euler request into a body-frame offset and resolve it
    // on the shared path, so both interfaces behave identically. roll/pitch/yaw are
    // degrees; Rotation takes a rotation vector in radians.
    MoveRequest req;
    req.relative = true;
    req.target.position.x = msg->x;
    req.target.position.y = msg->y;
    req.target.position.z = msg->z;
    double const deg2rad = M_PI / 180.0;
    Eigen::Vector3d const rpy_rad{ msg->roll * deg2rad, msg->pitch * deg2rad, msg->yaw * deg2rad };
    req.target.orientation = mil::geometry::Rotation{ rpy_rad }.quat_msg();
    req.precise = msg->precise;
    req.command_id = msg->command_id;
    drive_to_(req);
}

void MovementManager::drive_to_(MoveRequest const &req)
{
    using subjugator_msgs::msg::MoveStatus;

    // If a move is still in flight, tell any waiter it was superseded before we
    // repoint the goal (so they fail fast instead of waiting on a stale target).
    if (have_active_command_ && active_state_ == MoveStatus::ACTIVE)
    {
        double dist = 0.0, ang = 0.0;
        current_errors_(active_goal_, dist, ang);
        active_state_ = MoveStatus::PREEMPTED;
        emit_status_(MoveStatus::PREEMPTED, "preempted by move #" + std::to_string(req.command_id), dist, ang);
        RCLCPP_WARN(get_logger(), "Move #%u PREEMPTED by #%u", active_command_id_, req.command_id);
    }

    geometry_msgs::msg::Pose const goal = resolve_goal_(req);
    goal_pub_->publish(goal);

    // Size this move's tolerances and timeout from how far it starts from the goal
    double pos_err0 = 0.0, ori_err0 = 0.0;
    current_errors_(goal, pos_err0, ori_err0);
    double const ori_term = std::min(ori_err0, 5.0);
    if (req.precise)
    {
        active_tight_pos_ = precise_pos_tol;
        active_tight_ori_ = precise_ori_tol;
        active_loose_pos_ = pos_tol;
        active_loose_ori_ = ori_tol;
        active_timeout_s_ = 10.0 * pos_err0 + 2.0 * ori_term + 15.0;
    }
    else
    {
        active_tight_pos_ = pos_tol;
        active_tight_ori_ = ori_tol;
        active_loose_pos_ = loose_pos_tol;
        active_loose_ori_ = loose_ori_tol;
        active_timeout_s_ = 4.5 * pos_err0 + ori_term + 4.0;
    }

    // Latch as the last goal (relative moves chain off it) and start supervising
    last_goal_ = goal;
    active_goal_ = goal;
    active_command_id_ = req.command_id;
    active_precise_ = req.precise;
    active_state_ = MoveStatus::ACTIVE;
    active_start_ = now();
    have_active_command_ = true;

    RCLCPP_INFO(get_logger(), "Move #%u (%s, %s) -> goal pos(%.2f, %.2f, %.2f), timeout %.1fs", req.command_id,
                req.relative ? "relative" : "absolute", req.precise ? "precise" : "regular", goal.position.x,
                goal.position.y, goal.position.z, active_timeout_s_);
}

void MovementManager::publish_status_()
{
    using subjugator_msgs::msg::MoveStatus;

    if (!have_active_command_)
    {
        return;  // idle between moves
    }

    double dist = 0.0, ang = 0.0;
    if (!current_errors_(active_goal_, dist, ang))
    {
        return;  // no odom right now; can't evaluate arrival
    }

    rclcpp::Time const t = now();
    rclcpp::Time const deadline = active_start_ + rclcpp::Duration::from_seconds(active_timeout_s_);
    rclcpp::Time const grace_deadline = deadline + rclcpp::Duration::from_seconds(grace_secs);
    bool const in_grace = t >= deadline;

    // Which tolerance applies right now: tight before the deadline, the relaxed one
    // during the grace window.
    bool const within = in_grace ? (dist <= active_loose_pos_ && ang <= active_loose_ori_) :
                                   (dist <= active_tight_pos_ && ang <= active_tight_ori_);

    uint8_t state = MoveStatus::ACTIVE;
    std::string message;

    if (t >= grace_deadline)
    {
        state = MoveStatus::TIMED_OUT;
        message = "did not reach loose tolerance within grace period";
    }
    else if (within)
    {
        state = in_grace ? MoveStatus::REACHED_LOOSE : MoveStatus::REACHED;
        if (in_grace)
        {
            message = "reached only loose tolerance after timeout (degraded success)";
        }
    }
    else if (in_grace)
    {
        message = "grace period: seeking loose tolerance";
    }

    active_state_ = state;
    emit_status_(state, message, dist, ang);

    if (state != MoveStatus::ACTIVE)
    {
        char const *kind = active_precise_ ? "precise" : "regular";
        switch (state)
        {
            case MoveStatus::REACHED:
                RCLCPP_INFO(get_logger(), "Move #%u [%s] REACHED (dist %.2f m, ang %.1f deg)", active_command_id_, kind,
                            dist, ang);
                break;
            case MoveStatus::REACHED_LOOSE:
                RCLCPP_WARN(get_logger(), "Move #%u [%s] REACHED_LOOSE (dist %.2f m, ang %.1f deg) - %s",
                            active_command_id_, kind, dist, ang, message.c_str());
                break;
            case MoveStatus::TIMED_OUT:
                RCLCPP_WARN(get_logger(), "Move #%u [%s] TIMED_OUT (dist %.2f m, ang %.1f deg)", active_command_id_,
                            kind, dist, ang);
                break;
            default:
                break;
        }
        // Terminal status emitted; go idle until the next move.
        have_active_command_ = false;
    }
}
