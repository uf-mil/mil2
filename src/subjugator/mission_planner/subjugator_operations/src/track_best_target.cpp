#include "track_best_target.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

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

BT::PortsList TrackBestTarget::providedPorts()
{
    BT::PortsList ports;

    ports.insert(BT::InputPort<std::string>("label", "torpedoTarget", "YOLO class label to track"));
    ports.insert(BT::InputPort<double>("min_conf", 0.15, "Minimum detection confidence"));
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));

    ports.insert(BT::OutputPort<double>("best_qx"));
    ports.insert(BT::OutputPort<double>("best_qy"));
    ports.insert(BT::OutputPort<double>("best_qz"));
    ports.insert(BT::OutputPort<double>("best_qw"));

    return ports;
}

BT::NodeStatus TrackBestTarget::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;

    best_score_ = -1e18;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TrackBestTarget::onRunning()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;

    std::string label = "torpedoTarget";
    double min_conf = 0.15;
    (void)getInput("label", label);
    (void)getInput("min_conf", min_conf);

    // Snapshot latest detections (copy optional)
    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }

    if (!arr || arr->detections.empty())
        return BT::NodeStatus::RUNNING;

    // Largest matching detection this frame, scored by bbox area
    auto area_of = [](yolo_msgs::msg::Detection const& d) -> double
    { return std::max(0.0, d.bbox.size.x) * std::max(0.0, d.bbox.size.y); };

    double best_score_frame = -1e18;
    bool found = false;

    for (auto const& d : arr->detections)
    {
        if (d.class_name != label || d.score < min_conf)
            continue;

        double const score = area_of(d);
        if (score > best_score_frame)
        {
            best_score_frame = score;
            found = true;
        }
    }

    if (!found)
        return BT::NodeStatus::RUNNING;

    // If this frame is the best we've ever seen, snapshot orientation and output it
    if (best_score_frame > best_score_)
    {
        geometry_msgs::msg::Pose current{};
        {
            std::scoped_lock lk(ctx_->odom_mx);
            if (!ctx_->latest_odom)
                return BT::NodeStatus::RUNNING;
            current = ctx_->latest_odom->pose.pose;
        }

        geometry_msgs::msg::Quaternion q = current.orientation;
        normalize_quat(q);

        best_score_ = best_score_frame;

        setOutput("best_qx", q.x);
        setOutput("best_qy", q.y);
        setOutput("best_qz", q.z);
        setOutput("best_qw", q.w);

        RCLCPP_INFO(ctx_->logger(), "TrackBestTarget: new best '%s' (area %.1f px^2), orientation saved", label.c_str(),
                    best_score_);
    }

    return BT::NodeStatus::RUNNING;
}

void TrackBestTarget::onHalted()
{
}
