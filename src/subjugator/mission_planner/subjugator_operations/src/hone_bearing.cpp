#include "hone_bearing.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "quat_math.hpp"
#include "select_target_logic.hpp"  // reuse parse_labels (already unit-tested)

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

HoneBearing::HoneBearing(std::string const& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList HoneBearing::providedPorts()
{
    return { BT::InputPort<std::string>("label", "shark", "Target label (single)"),
             BT::InputPort<std::string>("labels", "",
                                        "CSV of classes; if set, hone the best detection among them (overrides label)"),
             BT::InputPort<double>("offset_deg", 0.0, "Positive=left yaw bias, negative=right"),
             BT::InputPort<double>("fov_deg", 110.0, "Horizontal FOV of camera"),
             BT::InputPort<double>("min_conf", 0.30, "Minimum confidence"),
             BT::InputPort<bool>("wait_for_detection", false,
                                 "If true, return RUNNING (not FAILURE) while no matching detection is present, so "
                                 "an enclosing Repeat/Timeout can wait for the image instead of aborting"),
             BT::InputPort<std::shared_ptr<Context>>("ctx") };
}

BT::NodeStatus HoneBearing::onStart()
{
    if (!require_ctx(*this, ctx_, "HoneBearing"))
    {
        return BT::NodeStatus::FAILURE;
    }
    published_ = false;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoneBearing::onRunning()
{
    if (published_)
    {
        return BT::NodeStatus::SUCCESS;
    }

    // Inputs
    std::string label;
    double offset_deg = 0.0;
    double fov = 110.0;
    double min_conf = 0.30;

    (void)getInput("label", label);
    (void)getInput("offset_deg", offset_deg);
    (void)getInput("fov_deg", fov);
    (void)getInput("min_conf", min_conf);

    std::string labels_csv;
    (void)getInput("labels", labels_csv);
    // A non-empty `labels` port selects multi-label mode -- even if it parses to no
    // classes (e.g. whitespace/separators only). Such a value must match nothing, not
    // silently fall back to the single-label `label` default ("shark").
    bool const use_labels = !labels_csv.empty();
    std::vector<std::string> const label_set =
        use_labels ? select_target::parse_labels(labels_csv) : std::vector<std::string>{};
    if (use_labels && label_set.empty() && !warned_empty_labels_)
    {
        RCLCPP_WARN(ctx_->logger(), "HoneBearing: 'labels'='%s' parsed to no classes; will match nothing",
                    labels_csv.c_str());
        warned_empty_labels_ = true;
    }

    // When honing an image that may not be in view yet (S7), treat "not ready" (no
    // image size, no detections, no matching class) as RUNNING so an enclosing
    // Repeat/Timeout keeps waiting; a transient miss no longer aborts the stage.
    bool wait_for_detection = false;
    (void)getInput("wait_for_detection", wait_for_detection);
    BT::NodeStatus const not_ready = wait_for_detection ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;

    // Get image width
    uint32_t W = 0, H = 0;
    ctx_->image_size_for("front", W, H);

    if (W == 0)
    {
        if (!wait_for_detection)
            RCLCPP_WARN(ctx_->logger(), "HoneBearing: image size unknown");
        return not_ready;
    }

    // Best detection
    std::optional<yolo_msgs::msg::DetectionArray> arr = ctx_->detections_for("front");

    if (!arr || arr->detections.empty())
    {
        return not_ready;
    }

    yolo_msgs::msg::Detection const* best = nullptr;
    double best_conf = 0.0;
    for (auto const& d : arr->detections)
    {
        bool const matches = use_labels ?
                                 (std::find(label_set.begin(), label_set.end(), d.class_name) != label_set.end()) :
                                 (d.class_name == label);
        if (matches && d.score >= min_conf && d.score > best_conf)
        {
            best = &d;
            best_conf = d.score;
        }
    }

    if (!best)
    {
        return not_ready;
    }

    // Pixel -> bearing (deg). Right positive.
    double cx = best->bbox.center.position.x;
    double bearing = ((cx - static_cast<double>(W) / 2.0) / (static_cast<double>(W) / 2.0)) * (fov / 2.0);

    // Turn command (deg). Left positive.
    double yaw_cmd_deg = offset_deg - bearing;

    geometry_msgs::msg::Pose current{};
    {
        std::scoped_lock lk(ctx_->odom_mx);
        if (ctx_->latest_odom)
        {
            current = ctx_->latest_odom->pose.pose;
        }
    }

    // Compose absolute target orientation: q_out = q_cur * yaw_delta(yaw_cmd_deg)
    geometry_msgs::msg::Pose out = current;
    out.orientation = quat_math::multiply(current.orientation, quat_math::yaw_delta(yaw_cmd_deg));
    quat_math::normalize(out.orientation);

    // Publish exactly once
    ctx_->command_goal(out);
    published_ = true;

    RCLCPP_INFO(ctx_->logger(), "HoneBearing: bearing=%.1f°, offset=%.1f°, cmd=%.1f°", bearing, offset_deg,
                yaw_cmd_deg);

    return BT::NodeStatus::SUCCESS;
}

void HoneBearing::onHalted()
{
}
