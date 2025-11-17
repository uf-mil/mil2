#include "determine_channel_side.hpp"

#include <algorithm>

#include <yolo_msgs/msg/detection_array.hpp>

DetermineChannelSide::DetermineChannelSide(std::string const& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList DetermineChannelSide::providedPorts()
{
    BT::PortsList ports;
    ports.insert(BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"));
    ports.insert(BT::InputPort<int>("consecutive_frames", 3, "Decision stability frames"));
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));

    ports.insert(BT::OutputPort<std::string>("channel_side"));  // "right" or "left"
    ports.insert(BT::OutputPort<bool>("require_red_left"));     // true if right channel
    ports.insert(BT::OutputPort<bool>("is_right"));             // same as above, convenience
    return ports;
}

BT::NodeStatus DetermineChannelSide::onStart()
{
    right_count_ = left_count_ = 0;
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;
    (void)getInput("min_conf", min_conf_);
    (void)getInput("consecutive_frames", need_frames_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetermineChannelSide::onRunning()
{
    // need image first so pixel centers are meaningful
    {
        std::scoped_lock lk(ctx_->img_mx);
        if (ctx_->img_width == 0)
            return BT::NodeStatus::RUNNING;
    }

    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }
    if (!arr || arr->detections.empty())
        return BT::NodeStatus::RUNNING;

    // pick largest red and largest white by area
    yolo_msgs::msg::Detection const* best_r = nullptr;
    double ar = 0.0;
    yolo_msgs::msg::Detection const* best_w = nullptr;
    double aw = 0.0;

    for (auto const& d : arr->detections)
    {
        if (d.score < min_conf_)
            continue;
        double area = std::max(0.0, d.bbox.size.x) * std::max(0.0, d.bbox.size.y);
        if (d.class_name == "red-pole" && area > ar)
        {
            ar = area;
            best_r = &d;
        }
        if (d.class_name == "white-pole" && area > aw)
        {
            aw = area;
            best_w = &d;
        }
    }
    if (!best_r || !best_w)
        return BT::NodeStatus::RUNNING;

    bool const right = (best_w->bbox.center.position.x > best_r->bbox.center.position.x);
    if (right)
    {
        right_count_++;
        left_count_ = 0;
    }
    else
    {
        left_count_++;
        right_count_ = 0;
    }

    if (right_count_ >= need_frames_)
    {
        setOutput("channel_side", std::string("right"));
        setOutput("require_red_left", true);
        setOutput("is_right", true);
        RCLCPP_INFO_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 800, "DetermineChannelSide: RIGHT channel");
        return BT::NodeStatus::SUCCESS;
    }
    if (left_count_ >= need_frames_)
    {
        setOutput("channel_side", std::string("left"));
        setOutput("require_red_left", false);
        setOutput("is_right", false);
        RCLCPP_INFO_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 800, "DetermineChannelSide: LEFT channel");
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}
