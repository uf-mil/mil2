#include "board_arch_step.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <yolo_msgs/msg/detection_array.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace
{
// Rotate body-frame vector (rx,ry,rz) into world frame using pose orientation.
// Same math as PublishGoalPose::rotateVectorByQuat.
void rotateBodyToWorld(geometry_msgs::msg::Pose const& ref, double rx, double ry, double rz, double& ox, double& oy,
                       double& oz)
{
    auto const& q = ref.orientation;
    double w1 = -q.x * rx - q.y * ry - q.z * rz;
    double x1 = q.w * rx + q.y * rz - q.z * ry;
    double y1 = q.w * ry + q.z * rx - q.x * rz;
    double z1 = q.w * rz + q.x * ry - q.y * rx;

    ox = x1 * q.w - w1 * q.x - y1 * q.z + z1 * q.y;
    oy = y1 * q.w - w1 * q.y - z1 * q.x + x1 * q.z;
    oz = z1 * q.w - w1 * q.z - x1 * q.y + y1 * q.x;
}

// Find keypoint by 1-based id with sufficient confidence; returns nullptr if absent.
yolo_msgs::msg::KeyPoint2D const* findKeypoint(yolo_msgs::msg::Detection const& det, int id, double min_kp_conf)
{
    for (auto const& kp : det.keypoints.data)
    {
        if (kp.id == id && kp.score >= min_kp_conf)
            return &kp;
    }
    return nullptr;
}
}  // namespace

BoardArchStep::BoardArchStep(std::string const& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList BoardArchStep::providedPorts()
{
    return {
        BT::InputPort<std::string>("board_label", "torpedoTarget", "YOLO class carrying the 4 corner keypoints"),
        BT::InputPort<double>("min_conf", 0.30, "Minimum board detection confidence"),
        BT::InputPort<double>("min_kp_conf", 0.15, "Minimum per-keypoint confidence"),
        BT::InputPort<double>("radius_m", 2.0, "Assumed distance to board / orbit radius (m)"),
        BT::InputPort<double>("step_deg", 5.0, "Orbit arc angle commanded per step (deg)"),
        BT::InputPort<double>("deadband_frac", 0.08, "|left-right|/mean edge below this counts as head-on"),
        BT::InputPort<double>("dir_sign", 1.0, "Arch direction sign; set -1 to invert"),
        BT::InputPort<int>("max_steps", 8, "Max orbit steps before giving up (caps total arch)"),
        BT::InputPort<double>("pos_tol", 0.15, "Arrival position tolerance (m)"),
        BT::InputPort<double>("ori_tol_deg", 5.0, "Arrival orientation tolerance (deg)"),
        BT::InputPort<std::shared_ptr<Context>>("ctx"),
        // Diagnostics (optional to bind in XML)
        BT::OutputPort<double>("left_edge_px"),
        BT::OutputPort<double>("right_edge_px"),
        BT::OutputPort<double>("edge_imbalance_frac"),
    };
}

BT::NodeStatus BoardArchStep::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "BoardArchStep: missing ctx");
        return BT::NodeStatus::FAILURE;
    }
    waiting_for_goal_ = false;
    steps_done_ = 0;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BoardArchStep::onRunning()
{
    std::string board_label = "torpedoTarget";
    double min_conf = 0.30, min_kp_conf = 0.15, radius_m = 2.0, step_deg = 5.0;
    double deadband_frac = 0.08, dir_sign = 1.0, pos_tol = 0.15, ori_tol_deg = 5.0;
    int max_steps = 8;
    getInput("board_label", board_label);
    getInput("min_conf", min_conf);
    getInput("min_kp_conf", min_kp_conf);
    getInput("radius_m", radius_m);
    getInput("step_deg", step_deg);
    getInput("deadband_frac", deadband_frac);
    getInput("dir_sign", dir_sign);
    getInput("max_steps", max_steps);
    getInput("pos_tol", pos_tol);
    getInput("ori_tol_deg", ori_tol_deg);

    // Wait for the previously commanded orbit step to finish before re-evaluating.
    if (waiting_for_goal_)
    {
        geometry_msgs::msg::Pose cur{};
        {
            std::scoped_lock lk(ctx_->odom_mx);
            if (ctx_->latest_odom)
                cur = ctx_->latest_odom->pose.pose;
        }
        double dx = cur.position.x - pending_goal_.position.x;
        double dy = cur.position.y - pending_goal_.position.y;
        double dz = cur.position.z - pending_goal_.position.z;
        double pos_err = std::sqrt(dx * dx + dy * dy + dz * dz);

        auto const& a = cur.orientation;
        auto const& b = pending_goal_.orientation;
        double dot = std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
        dot = std::clamp(dot, 0.0, 1.0);
        double ori_err_deg = 2.0 * std::acos(dot) * 180.0 / M_PI;

        if (pos_err > pos_tol || ori_err_deg > ori_tol_deg)
            return BT::NodeStatus::RUNNING;

        waiting_for_goal_ = false;
        ++steps_done_;
    }

    // Find the highest-confidence board detection that exposes all 4 corners.
    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }

    yolo_msgs::msg::Detection const* board = nullptr;
    double best_conf = 0.0;
    if (arr)
    {
        for (auto const& d : arr->detections)
        {
            if (d.class_name == board_label && d.score >= min_conf && d.score > best_conf)
            {
                board = &d;
                best_conf = d.score;
            }
        }
    }

    if (!board)
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000,
                             "BoardArchStep: no '%s' detection, holding", board_label.c_str());
        return BT::NodeStatus::RUNNING;
    }

    auto const* tl = findKeypoint(*board, 1, min_kp_conf);  // top-left
    auto const* tr = findKeypoint(*board, 2, min_kp_conf);  // top-right
    auto const* bl = findKeypoint(*board, 3, min_kp_conf);  // bottom-left
    auto const* br = findKeypoint(*board, 4, min_kp_conf);  // bottom-right
    if (!tl || !tr || !bl || !br)
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000,
                             "BoardArchStep: board found but <4 corner keypoints visible, holding");
        return BT::NodeStatus::RUNNING;
    }

    auto edge_len = [](yolo_msgs::msg::KeyPoint2D const* p, yolo_msgs::msg::KeyPoint2D const* q)
    {
        double dx = p->point.x - q->point.x;
        double dy = p->point.y - q->point.y;
        return std::sqrt(dx * dx + dy * dy);
    };
    double const left_edge = edge_len(tl, bl);   // kp1 -> kp3
    double const right_edge = edge_len(tr, br);  // kp2 -> kp4
    double const mean = 0.5 * (left_edge + right_edge);
    if (mean < 1e-6)
        return BT::NodeStatus::RUNNING;  // degenerate, wait for a better frame

    double const imbalance = (left_edge - right_edge) / mean;  // >0 => left edge longer
    setOutput("left_edge_px", left_edge);
    setOutput("right_edge_px", right_edge);
    setOutput("edge_imbalance_frac", imbalance);

    if (std::abs(imbalance) <= deadband_frac)
    {
        RCLCPP_INFO(ctx_->logger(), "BoardArchStep: head-on (imbalance=%.3f <= %.3f) after %d step(s)", imbalance,
                    deadband_frac, steps_done_);
        return BT::NodeStatus::SUCCESS;
    }

    if (steps_done_ >= max_steps)
    {
        RCLCPP_WARN(ctx_->logger(), "BoardArchStep: hit max_steps=%d with imbalance=%.3f; proceeding anyway", max_steps,
                    imbalance);
        return BT::NodeStatus::SUCCESS;
    }

    // Orbit by one small step. imbalance>0 (left edge longer) -> arch right -> phi>0.
    // Geometry (board straight ahead at radius_m, body x forward, y left):
    //   forward dx = R(1-cos phi), left dy = -R sin phi, yaw = +phi (re-face board).
    double const phi_deg = dir_sign * ((imbalance > 0.0) ? step_deg : -step_deg);
    double const phi = phi_deg * M_PI / 180.0;

    double const rel_x = radius_m * (1.0 - std::cos(phi));  // forward
    double const rel_y = -radius_m * std::sin(phi);         // +y is left
    double const rel_z = 0.0;

    geometry_msgs::msg::Pose cur{};
    {
        std::scoped_lock lk(ctx_->odom_mx);
        if (!ctx_->latest_odom)
            return BT::NodeStatus::RUNNING;
        cur = ctx_->latest_odom->pose.pose;
    }

    // Position: current + body->world rotation of (rel_x, rel_y, rel_z).
    double wx = 0.0, wy = 0.0, wz = 0.0;
    rotateBodyToWorld(cur, rel_x, rel_y, rel_z, wx, wy, wz);

    geometry_msgs::msg::Pose goal = cur;
    goal.position.x = cur.position.x + wx;
    goal.position.y = cur.position.y + wy;
    goal.position.z = cur.position.z + wz;

    // Orientation: cur * yaw_delta(phi). Body-frame yaw about +Z (same as AlignYaw).
    geometry_msgs::msg::Quaternion dq{};
    dq.z = std::sin(phi / 2.0);
    dq.w = std::cos(phi / 2.0);

    auto const& c = cur.orientation;
    auto& o = goal.orientation;
    o.x = c.w * dq.x + c.x * dq.w + c.y * dq.z - c.z * dq.y;
    o.y = c.w * dq.y - c.x * dq.z + c.y * dq.w + c.z * dq.x;
    o.z = c.w * dq.z + c.x * dq.y - c.y * dq.x + c.z * dq.w;
    o.w = c.w * dq.w - c.x * dq.x - c.y * dq.y - c.z * dq.z;
    double n = std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w);
    if (n > 1e-12)
    {
        o.x /= n;
        o.y /= n;
        o.z /= n;
        o.w /= n;
    }

    ctx_->goal_pub->publish(goal);
    {
        std::scoped_lock lk(ctx_->last_goal_mx);
        ctx_->last_goal = goal;
    }
    pending_goal_ = goal;
    waiting_for_goal_ = true;

    RCLCPP_INFO(ctx_->logger(),
                "BoardArchStep: imbalance=%.3f (L=%.0f R=%.0f) -> arch %s phi=%.1f deg "
                "(dx=%.2f dy=%.2f) step %d/%d",
                imbalance, left_edge, right_edge, phi_deg > 0 ? "right" : "left", phi_deg, rel_x, rel_y,
                steps_done_ + 1, max_steps);
    return BT::NodeStatus::RUNNING;
}

void BoardArchStep::onHalted()
{
    waiting_for_goal_ = false;
}
