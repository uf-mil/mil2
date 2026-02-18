#include "any_poles_detected.hpp"

#include <yolo_msgs/msg/detection_array.hpp>

BT::PortsList AnyPolesDetected::providedPorts()
{
    BT::PortsList ports;
    ports.insert(BT::InputPort<double>("min_conf", 0.25, "Minimum detection confidence"));
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));
    return ports;
}

BT::NodeStatus AnyPolesDetected::tick()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;
    double min_conf = 0.30;
    (void)getInput("min_conf", min_conf);

    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }
    if (!arr)
        return BT::NodeStatus::FAILURE;

    for (auto const& d : arr->detections)
    {
        if (d.score < min_conf)
            continue;
        if (d.class_name == "red-pole" || d.class_name == "white-pole")
            return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}
