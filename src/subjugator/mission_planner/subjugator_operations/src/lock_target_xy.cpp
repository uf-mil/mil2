#include "lock_target_xy.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

namespace
{
// Parse "x,y,z" into a Vec3, falling back on parse failure.
target_projection::Vec3 parse_vec3(std::string const& s, target_projection::Vec3 fallback)
{
    double a = 0.0, b = 0.0, c = 0.0;
    if (std::sscanf(s.c_str(), "%lf,%lf,%lf", &a, &b, &c) == 3)
    {
        return { a, b, c };
    }
    return fallback;
}

// Rotate a body-frame vector into world by the sub's orientation quaternion:
// v' = v + 2w(u x v) + 2(u x (u x v)),  u = (x,y,z).
target_projection::Vec3 rotate_by_quat(geometry_msgs::msg::Quaternion const& q, target_projection::Vec3 v)
{
    double const ux = q.x, uy = q.y, uz = q.z, w = q.w;
    double const cx = uy * v.z - uz * v.y;
    double const cy = uz * v.x - ux * v.z;
    double const cz = ux * v.y - uy * v.x;
    double const ccx = uy * cz - uz * cy;
    double const ccy = uz * cx - ux * cz;
    double const ccz = ux * cy - uy * cx;
    return { v.x + 2.0 * w * cx + 2.0 * ccx, v.y + 2.0 * w * cy + 2.0 * ccy, v.z + 2.0 * w * cz + 2.0 * ccz };
}
}  // namespace

LockTargetXY::LockTargetXY(std::string const& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList LockTargetXY::providedPorts()
{
    return { BT::InputPort<std::string>("label", "table", "Target class to center on"),
             BT::InputPort<std::string>("camera", "down", "Detection stream: 'front' or 'down'"),
             BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"),
             BT::InputPort<double>("tol_norm", 0.05, "Centered when |ex|,|ey| < tol"),
             BT::InputPort<int>("settle_ticks", 3, "Consecutive fresh centered frames for SUCCESS"),
             BT::InputPort<int>("miss_frames", 5, "Consecutive fresh misses before FAILURE"),
             BT::InputPort<double>("table_z", 0.0, "Table-top world z (m) the ray is cast onto"),
             BT::InputPort<double>("hfov", 1.919862177, "Camera horizontal FOV (rad); URDF default"),
             // Static base_link->down_cam optical basis expressed in base_link.
             // Origin from sub9.urdf.xacro down_cam (xyz=0.45222 0.027913 -0.22,
             // rpy=0 1.57 0). The image-axis directions match CenterCamera's
             // sim-tuned pixel->body mapping (swap_axes=false, +x/+y): image +x
             // (target right) => the target lies toward body +x, image +y (target
             // down) => body +y. Re-confirm/flip with the sim geometry check.
             BT::InputPort<std::string>("cam_offset", "0.45222,0.027913,-0.22", "Camera origin in base_link x,y,z"),
             BT::InputPort<std::string>("cam_right", "1,0,0", "Image +x dir in base_link x,y,z"),
             BT::InputPort<std::string>("cam_down", "0,1,0", "Image +y dir in base_link x,y,z"),
             BT::InputPort<std::string>("cam_forward", "0,0,-1", "Optical view dir in base_link x,y,z"),
             BT::InputPort<double>("gripper_x", 0.0, "Gripper x offset in base_link (m)"),
             BT::InputPort<double>("gripper_y", 0.0, "Gripper y offset in base_link (m)"),
             BT::InputPort<double>("ema_alpha", 0.3, "World-XY smoothing (0-1, higher = faster)"),
             BT::InputPort<double>("max_step", 0.25, "Per-cycle goal slew clamp (m)"),
             BT::InputPort<std::shared_ptr<Context>>("ctx") };
}

BT::NodeStatus LockTargetXY::onStart()
{
    if (!require_ctx(*this, ctx_, "LockTargetXY"))
    {
        return BT::NodeStatus::FAILURE;
    }
    in_tol_count_ = 0;
    have_estimate_ = false;
    std::string camera = "down";
    (void)getInput("camera", camera);
    gate_.seed_from(ctx_->detections_for(camera));
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LockTargetXY::onRunning()
{
    std::string label = "table", camera = "down";
    double min_conf = 0.30, tol_norm = 0.05, table_z = 0.0, hfov = 1.919862177;
    double gripper_x = 0.0, gripper_y = 0.0, ema_alpha = 0.3, max_step = 0.25;
    int settle_ticks = 3, miss_frames = 5;
    std::string cam_offset = "0.45222,0.027913,-0.22", cam_right = "1,0,0", cam_down = "0,1,0", cam_forward = "0,0,-1";

    (void)getInput("label", label);
    (void)getInput("camera", camera);
    (void)getInput("min_conf", min_conf);
    (void)getInput("tol_norm", tol_norm);
    (void)getInput("settle_ticks", settle_ticks);
    (void)getInput("miss_frames", miss_frames);
    (void)getInput("table_z", table_z);
    (void)getInput("hfov", hfov);
    (void)getInput("cam_offset", cam_offset);
    (void)getInput("cam_right", cam_right);
    (void)getInput("cam_down", cam_down);
    (void)getInput("cam_forward", cam_forward);
    (void)getInput("gripper_x", gripper_x);
    (void)getInput("gripper_y", gripper_y);
    (void)getInput("ema_alpha", ema_alpha);
    (void)getInput("max_step", max_step);

    std::optional<geometry_msgs::msg::Pose> current;
    {
        std::scoped_lock lk(ctx_->odom_mx);
        if (ctx_->latest_odom)
        {
            current = ctx_->latest_odom->pose.pose;
        }
    }
    if (!current)
    {
        return BT::NodeStatus::RUNNING;  // wait for odom
    }

    uint32_t W = 0, H = 0;
    if (!ctx_->image_size_for(camera, W, H))
    {
        return BT::NodeStatus::RUNNING;  // cold start
    }

    std::optional<yolo_msgs::msg::DetectionArray> arr = ctx_->detections_for(camera);
    if (!arr)
    {
        return BT::NodeStatus::RUNNING;
    }
    auto const* best = detection_gate::best_detection(*arr, label, min_conf);
    std::int64_t stamp_ns = rclcpp::Time(arr->header.stamp).nanoseconds();
    switch (gate_.update(best != nullptr, stamp_ns, miss_frames))
    {
        case detection_gate::MissGate::Verdict::kStale:
            return BT::NodeStatus::RUNNING;
        case detection_gate::MissGate::Verdict::kMiss:
            in_tol_count_ = 0;
            return BT::NodeStatus::RUNNING;
        case detection_gate::MissGate::Verdict::kLost:
            RCLCPP_WARN(ctx_->logger(), "LockTargetXY: '%s' lost (%d fresh frames without it)", label.c_str(),
                        gate_.misses);
            return BT::NodeStatus::FAILURE;
        case detection_gate::MissGate::Verdict::kHit:
            break;
    }

    double const ex = (best->bbox.center.position.x - static_cast<double>(W) / 2.0) / (static_cast<double>(W) / 2.0);
    double const ey = (best->bbox.center.position.y - static_cast<double>(H) / 2.0) / (static_cast<double>(H) / 2.0);

    if (std::abs(ex) < tol_norm && std::abs(ey) < tol_norm)
    {
        if (++in_tol_count_ >= settle_ticks)
        {
            RCLCPP_INFO(ctx_->logger(), "LockTargetXY: centered (ex=%.3f ey=%.3f)", ex, ey);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
    in_tol_count_ = 0;

    // Build the camera frame in world from odom + the static basis.
    auto const& q = current->orientation;
    target_projection::Vec3 const off_w = rotate_by_quat(q, parse_vec3(cam_offset, { 0.45222, 0.027913, -0.22 }));
    target_projection::CameraFrame cam;
    cam.origin = { current->position.x + off_w.x, current->position.y + off_w.y, current->position.z + off_w.z };
    cam.right = rotate_by_quat(q, parse_vec3(cam_right, { 1.0, 0.0, 0.0 }));
    cam.down = rotate_by_quat(q, parse_vec3(cam_down, { 0.0, 1.0, 0.0 }));
    cam.forward = rotate_by_quat(q, parse_vec3(cam_forward, { 0.0, 0.0, -1.0 }));

    auto const hit = target_projection::project_to_plane(ex, ey, hfov, static_cast<double>(H) / static_cast<double>(W),
                                                         cam, table_z);
    if (!hit)
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000,
                             "LockTargetXY: ray misses table plane, holding");
        return BT::NodeStatus::RUNNING;
    }

    if (!have_estimate_)
    {
        est_ = *hit;
        have_estimate_ = true;
    }
    else
    {
        est_.x += ema_alpha * (hit->x - est_.x);
        est_.y += ema_alpha * (hit->y - est_.y);
    }

    double const yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    target_projection::Vec2 base = target_projection::goal_base_xy(est_, yaw, gripper_x, gripper_y);

    // Slew clamp per cycle so a bad estimate can't command a large jump.
    double dx = base.x - current->position.x;
    double dy = base.y - current->position.y;
    double const dist = std::hypot(dx, dy);
    if (dist > max_step)
    {
        dx *= max_step / dist;
        dy *= max_step / dist;
    }

    geometry_msgs::msg::Pose goal = *current;  // holds z and orientation
    goal.position.x = current->position.x + dx;
    goal.position.y = current->position.y + dy;
    ctx_->command_goal(goal);

    RCLCPP_INFO(ctx_->logger(), "LockTargetXY: ex=%.3f ey=%.3f -> world(%.2f,%.2f) goal(%.2f,%.2f)", ex, ey, est_.x,
                est_.y, goal.position.x, goal.position.y);
    return BT::NodeStatus::RUNNING;
}

void LockTargetXY::onHalted()
{
}
