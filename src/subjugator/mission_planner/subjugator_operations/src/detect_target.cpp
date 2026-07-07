#include "detect_target.hpp"

#include <yolo_msgs/msg/detection_array.hpp>

BT::NodeStatus DetectTarget::onStart()
{
    if (!require_ctx(*this, ctx_, "DetectTarget"))
    {
        return BT::NodeStatus::FAILURE;
    }

    std::string label;
    (void)getInput("label", label);
    if (label.empty())
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "DetectTarget: empty label.");
        return BT::NodeStatus::FAILURE;
    }

    start_time_ = ctx_->node->now();
    // Only frames captured after this activation are evidence: the guard must
    // answer "is it visible NOW", not "was it visible in some cached frame".
    std::string camera = "front";
    (void)getInput("camera", camera);
    gate_.seed_from(ctx_->detections_for(camera));
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectTarget::onRunning()
{
    std::string label;
    double min_conf = 0.40;
    std::string camera = "front";
    int confirm_frames = 1;
    int miss_frames = 5;
    int timeout_msec = 5000;
    (void)getInput("label", label);
    (void)getInput("min_conf", min_conf);
    (void)getInput("camera", camera);
    (void)getInput("confirm_frames", confirm_frames);
    (void)getInput("miss_frames", miss_frames);
    (void)getInput("timeout_msec", timeout_msec);

    // Judge the current frame FIRST, timeout second: a decisive frame that
    // arrived just before expiry must not be discarded because the tick that
    // processes it lands past the deadline.
    if (std::optional<yolo_msgs::msg::DetectionArray> arr = ctx_->detections_for(camera))
    {
        switch (gate_.update(detection_gate::contains_label(*arr, label, min_conf),
                             detection_gate::MissGate::stamp_ns_of(*arr), miss_frames))
        {
            case detection_gate::MissGate::Verdict::kStale:
            case detection_gate::MissGate::Verdict::kMiss:
                break;  // no verdict yet; fall through to the timeout bound
            case detection_gate::MissGate::Verdict::kLost:
                return BT::NodeStatus::FAILURE;
            case detection_gate::MissGate::Verdict::kHit:
                if (gate_.hits >= confirm_frames)
                {
                    return BT::NodeStatus::SUCCESS;
                }
                break;
        }
    }

    // Bound the wait: with a dead/silent stream the gate would stay stale
    // forever. FAILURE mirrors the old no-detections behavior, just slower.
    if ((ctx_->node->now() - start_time_).seconds() * 1000.0 > timeout_msec)
    {
        RCLCPP_WARN(ctx_->logger(), "DetectTarget: no verdict on '%s' within %d ms; failing", label.c_str(),
                    timeout_msec);
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}
