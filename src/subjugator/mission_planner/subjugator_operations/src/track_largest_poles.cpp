#include "track_largest_poles.hpp"

#include <algorithm>
#include <cmath>

#include <geometry_msgs/msg/pose.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline void normalize_quat(geometry_msgs::msg::Quaternion& q)
{
    double const n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (n < 1e-12)
    {
        q.x = q.y = q.z = 0.0;
        q.w = 1.0;
        return;
    }
    q.x /= n;
    q.y /= n;
    q.z /= n;
    q.w /= n;
}

// yaw delta around +Z in BODY frame (matches how your relative yaw is applied)
static inline geometry_msgs::msg::Quaternion yaw_delta_quat(double yaw_deg)
{
    double const r = (yaw_deg * M_PI / 180.0) * 0.5;
    geometry_msgs::msg::Quaternion q{};
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(r);
    q.w = std::cos(r);
    return q;
}

static inline geometry_msgs::msg::Quaternion quat_multiply(geometry_msgs::msg::Quaternion const& a,
                                                           geometry_msgs::msg::Quaternion const& b)
{
    geometry_msgs::msg::Quaternion out;
    out.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    out.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    out.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    out.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return out;
}

BT::PortsList TrackLargestPoles::providedPorts()
{
    BT::PortsList ports;
    // Inputs
    ports.insert(BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"));
    ports.insert(BT::InputPort<double>("fov_deg", 110.0, "Camera horizontal FOV (deg) for bearing calc"));
    ports.insert(BT::InputPort<bool>("require_red_left", false, "Assume red is left of white"));
    ports.insert(BT::InputPort<bool>("reset_on_start", true, "Reset memorized best on start"));
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));

    // Legacy outputs (largest-by-area ever seen)
    ports.insert(BT::OutputPort<double>("red_cx_px"));
    ports.insert(BT::OutputPort<double>("red_cy_px"));
    ports.insert(BT::OutputPort<double>("red_w_px"));
    ports.insert(BT::OutputPort<double>("red_h_px"));
    ports.insert(BT::OutputPort<double>("white_cx_px"));
    ports.insert(BT::OutputPort<double>("white_cy_px"));
    ports.insert(BT::OutputPort<double>("white_w_px"));
    ports.insert(BT::OutputPort<double>("white_h_px"));

    // Best gap snapshot (absolute orientation + score + bearing at snapshot)
    ports.insert(BT::OutputPort<double>("best_qx"));
    ports.insert(BT::OutputPort<double>("best_qy"));
    ports.insert(BT::OutputPort<double>("best_qz"));
    ports.insert(BT::OutputPort<double>("best_qw"));
    ports.insert(BT::OutputPort<double>("best_score"));
    ports.insert(BT::OutputPort<double>("best_bearing_deg"));
    ports.insert(BT::OutputPort<double>("best_mid_cx_px"));

    ports.insert(BT::OutputPort<double>("best_target_qx"));
    ports.insert(BT::OutputPort<double>("best_target_qy"));
    ports.insert(BT::OutputPort<double>("best_target_qz"));
    ports.insert(BT::OutputPort<double>("best_target_qw"));
    ports.insert(BT::OutputPort<bool>("found_pair"));

    return ports;
}

BT::NodeStatus TrackLargestPoles::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;

    bool reset = true;
    (void)getInput("reset_on_start", reset);
    if (reset)
    {
        best_red_area_ = 0.0;
        best_white_area_ = 0.0;
    }

    best_pair_score_ = -1e18;
    setOutput("found_pair", false);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TrackLargestPoles::onRunning()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;

    double min_conf = 0.30, fov_deg = 110.0;
    bool require_red_left = false;
    (void)getInput("min_conf", min_conf);
    (void)getInput("fov_deg", fov_deg);
    (void)getInput("require_red_left", require_red_left);

    // Latest frame and image size
    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }
    if (!arr || arr->detections.empty())
        return BT::NodeStatus::RUNNING;

    uint32_t W = 0, H = 0;
    {
        std::scoped_lock lk(ctx_->img_mx);
        W = ctx_->img_width;
        H = ctx_->img_height;
    }
    if (W == 0)
        return BT::NodeStatus::RUNNING;

    // track single largest red/white by area (across time)
    for (auto const& det : arr->detections)
    {
        if (det.score < min_conf)
            continue;
        double w = det.bbox.size.x;
        double h = det.bbox.size.y;
        double area = std::max(0.0, w) * std::max(0.0, h);

        if (det.class_name == "red-pole" && area > best_red_area_)
        {
            best_red_area_ = area;
            setOutput("red_cx_px", det.bbox.center.position.x);
            setOutput("red_cy_px", det.bbox.center.position.y);
            setOutput("red_w_px", w);
            setOutput("red_h_px", h);
        }
        else if (det.class_name == "white-pole" && area > best_white_area_)
        {
            best_white_area_ = area;
            setOutput("white_cx_px", det.bbox.center.position.x);
            setOutput("white_cy_px", det.bbox.center.position.y);
            setOutput("white_w_px", w);
            setOutput("white_h_px", h);
        }
    }

    std::vector<yolo_msgs::msg::Detection const*> reds, whites;
    reds.reserve(arr->detections.size());
    whites.reserve(arr->detections.size());
    for (auto const& d : arr->detections)
    {
        if (d.score < min_conf)
            continue;
        if (d.class_name == "red-pole")
            reds.push_back(&d);
        if (d.class_name == "white-pole")
            whites.push_back(&d);
    }
    if (reds.empty() || whites.empty())
        return BT::NodeStatus::RUNNING;

    // Compute aread
    auto area_of = [](yolo_msgs::msg::Detection const* d)
    { return std::max(0.0, d->bbox.size.x) * std::max(0.0, d->bbox.size.y); };

    double const y_align_scale = 0.5 * std::max(1u, H);  // pixels
    double const center_weight = 0.5;                    // penalty for mid far from center

    yolo_msgs::msg::Detection const* best_r = nullptr;
    yolo_msgs::msg::Detection const* best_w = nullptr;
    double best_score_frame = -1e18;
    double best_mid_cx = 0.0;

    for (auto r : reds)
    {
        for (auto w : whites)
        {
            if (require_red_left && (r->bbox.center.position.x >= w->bbox.center.position.x))
                continue;

            double const ar = area_of(r);
            double const aw = area_of(w);
            double const pair_area = ar + aw;

            double const dy = std::abs(r->bbox.center.position.y - w->bbox.center.position.y);
            double const y_align = std::max(0.0, 1.0 - dy / y_align_scale);

            double const cx_mid = 0.5 * (r->bbox.center.position.x + w->bbox.center.position.x);
            double const center_pen = std::abs(cx_mid - double(W) / 2.0) / (double(W) / 2.0);

            double const score = pair_area * (1.0 + y_align) - center_weight * center_pen;

            if (score > best_score_frame)
            {
                best_score_frame = score;
                best_r = r;
                best_w = w;
                best_mid_cx = cx_mid;
            }
        }
    }

    if (!best_r || !best_w)
    {
        setOutput("found_pair", best_pair_score_ > -1e17);
        return BT::NodeStatus::RUNNING;
    }

    // Bearing of the midpoint (deg), using width+FOV fallback
    double const bearing_mid = ((best_mid_cx - double(W) / 2.0) / (double(W) / 2.0)) * (fov_deg / 2.0);

    // If this frame beats our memorized best, snapshot absolute orientation and compute absolute target
    if (best_score_frame > best_pair_score_)
    {
        best_pair_score_ = best_score_frame;

        geometry_msgs::msg::Pose cur{};
        {
            std::scoped_lock lk(ctx_->odom_mx);
            if (!ctx_->latest_odom)
                return BT::NodeStatus::RUNNING;
            cur = ctx_->latest_odom->pose.pose;
        }

        // Snapshot quaternion
        geometry_msgs::msg::Quaternion q_snap = cur.orientation;
        normalize_quat(q_snap);

        // Target quaternion = snapshot * yaw_delta(bearing_mid)
        geometry_msgs::msg::Quaternion q_delta = yaw_delta_quat(-bearing_mid);
        geometry_msgs::msg::Quaternion q_target = quat_multiply(q_snap, q_delta);
        normalize_quat(q_target);

        // Outputs
        setOutput("best_qx", q_snap.x);
        setOutput("best_qy", q_snap.y);
        setOutput("best_qz", q_snap.z);
        setOutput("best_qw", q_snap.w);

        setOutput("best_target_qx", q_target.x);
        setOutput("best_target_qy", q_target.y);
        setOutput("best_target_qz", q_target.z);
        setOutput("best_target_qw", q_target.w);

        setOutput("best_score", best_pair_score_);
        setOutput("best_bearing_deg", bearing_mid);
        setOutput("best_mid_cx_px", best_mid_cx);

        setOutput("found_pair", true);

        RCLCPP_INFO_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 800,
                             "TrackLargestPoles[mem]: updated best score=%.1f, bearing=%.1f deg", best_pair_score_,
                             bearing_mid);
    }

    return BT::NodeStatus::RUNNING;  // bounded by Timeout in the tree
}
