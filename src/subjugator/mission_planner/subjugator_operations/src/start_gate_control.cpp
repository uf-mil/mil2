#include "start_gate_control.hpp"

#include <cmath>
#include <optional>

#include <yolo_msgs/msg/detection_array.hpp>

StartGateControl::StartGateControl(std::string const& name, BT::NodeConfiguration const& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList StartGateControl::providedPorts()
{
    BT::PortsList ports;

    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));
    ports.insert(BT::InputPort<std::string>("label", "Search&Rescue", "Target YOLO class name"));
    ports.insert(BT::InputPort<double>("min_conf", 0.15, "Min YOLO confidence"));

    // The three control knobs. Error is normalized: +/-0.5 spans half the frame.
    ports.insert(BT::InputPort<double>("threshold", 0.05, "|err| below this = centered, no yaw"));
    ports.insert(BT::InputPort<double>("kp", 40.0, "Proportional gain (deg of yaw per unit err)"));
    ports.insert(BT::InputPort<double>("max_yaw_deg", 15.0, "Max abs yaw cmd per tick (deg)"));

    ports.insert(BT::OutputPort<double>("yaw_cmd", "Yaw step (deg) for this tick"));

    return ports;
}

BT::NodeStatus StartGateControl::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;

    centered_streak_ = 0;
    setOutput("yaw_cmd", 0.0);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StartGateControl::onRunning()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;

    std::string label = "Search&Rescue";
    double min_conf = 0.15;
    double threshold = 0.05;
    double kp = 40.0;
    double max_yaw_deg = 15.0;

    (void)getInput("label", label);
    (void)getInput("min_conf", min_conf);
    (void)getInput("threshold", threshold);
    (void)getInput("kp", kp);
    (void)getInput("max_yaw_deg", max_yaw_deg);

    uint32_t W = 0;
    {
        std::scoped_lock lk(ctx_->img_mx);
        W = ctx_->img_width;
    }

    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }

    // Largest (nearest) confident detection of the target class.
    yolo_msgs::msg::Detection const* best = nullptr;
    double best_h = -1.0;
    if (W > 0 && arr)
    {
        for (auto const& d : arr->detections)
        {
            if (d.class_name != label || d.score < min_conf)
                continue;
            double const h = std::max(0.0, d.bbox.size.y);
            if (h > best_h)
            {
                best_h = h;
                best = &d;
            }
        }
    }

    // No target this tick: command nothing and keep looping. The surrounding
    // Timeout is the escape hatch if it never appears.
    if (!best)
    {
        setOutput("yaw_cmd", 0.0);
        return BT::NodeStatus::SUCCESS;
    }

    // err > 0 = target right of center -> yaw clockwise (negative).
    double const err = best->bbox.center.position.x / static_cast<double>(W) - 0.5;

    if (std::abs(err) <= threshold)
    {
        setOutput("yaw_cmd", 0.0);
        if (++centered_streak_ >= kHoldTicks)
            return BT::NodeStatus::FAILURE;  // aligned — stop the loop
        return BT::NodeStatus::SUCCESS;
    }

    centered_streak_ = 0;
    setOutput("yaw_cmd", clamp(-kp * err, -max_yaw_deg, +max_yaw_deg));
    return BT::NodeStatus::SUCCESS;  // command computed — advance to RelativeMove
}

void StartGateControl::onHalted()
{
    centered_streak_ = 0;
    setOutput("yaw_cmd", 0.0);
}
