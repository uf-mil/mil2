#include "confirm_grasp_by_scale.hpp"

#include "detection_scale.hpp"

BT::NodeStatus ConfirmGraspByScale::onStart()
{
    if (!require_ctx(*this, ctx_, "ConfirmGraspByScale"))
    {
        return BT::NodeStatus::FAILURE;
    }

    start_time_ = ctx_->node->now();
    held_streak_ = 0;
    missed_streak_ = 0;
    // Seed with the currently cached frame: this node starts right after the
    // lift RelativeMove, and the cache can still hold a PRE-LIFT frame where a
    // missed object appears full-size. Judging that frame was the false-GRAB
    // bug; only frames captured after the lift count.
    std::string camera = "down";
    (void)getInput("camera", camera);
    gate_.seed_from(ctx_->detections_for(camera));
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ConfirmGraspByScale::onRunning()
{
    std::string label;
    std::string camera = "down";
    double min_conf = 0.30;
    double baseline = 0.0;
    double keep_ratio = 0.5;
    bool absent_is_grabbed = true;
    int confirm_frames = 3;
    int miss_frames = 5;
    int timeout_msec = 6000;
    (void)getInput("label", label);
    (void)getInput("camera", camera);
    (void)getInput("min_conf", min_conf);
    (void)getInput("baseline_area", baseline);
    (void)getInput("keep_ratio", keep_ratio);
    (void)getInput("absent_is_grabbed", absent_is_grabbed);
    (void)getInput("confirm_frames", confirm_frames);
    (void)getInput("miss_frames", miss_frames);
    (void)getInput("timeout_msec", timeout_msec);

    // No baseline (target was not seen before the lift, e.g. the down-cam model has not
    // landed): cannot verify -> stay open-loop and report success.
    if (baseline <= 0.0)
    {
        RCLCPP_WARN(ctx_->logger(), "ConfirmGraspByScale: no baseline for '%s'; assuming grasp (open-loop)",
                    label.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    // Judge the current frame FIRST, timeout second: a decisive frame that
    // arrived just before expiry must not be discarded because the tick that
    // processes it lands past the deadline.
    std::optional<yolo_msgs::msg::DetectionArray> arr = ctx_->detections_for(camera);
    if (!arr)
    {
        return timed_out_fallback(label, absent_is_grabbed, timeout_msec);
    }

    std::optional<double> current = best_bbox_area(*arr, label, min_conf);

    switch (gate_.update(current.has_value(), detection_gate::MissGate::stamp_ns_of(*arr), miss_frames))
    {
        case detection_gate::MissGate::Verdict::kStale:
            return timed_out_fallback(label, absent_is_grabbed, timeout_msec);  // same frame as last tick

        case detection_gate::MissGate::Verdict::kMiss:
            // Ambiguous frame (held object can be occluded; missed one can be
            // out of frame): breaks both streaks, policy applies only after
            // miss_frames in a row (kLost below).
            held_streak_ = 0;
            missed_streak_ = 0;
            return timed_out_fallback(label, absent_is_grabbed, timeout_msec);

        case detection_gate::MissGate::Verdict::kLost:
            RCLCPP_WARN(ctx_->logger(), "ConfirmGraspByScale: '%s' not detected on %d consecutive frames; %s",
                        label.c_str(), gate_.misses, absent_is_grabbed ? "assuming grabbed" : "assuming missed");
            return absent_is_grabbed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

        case detection_gate::MissGate::Verdict::kHit:
        {
            double const ratio = *current / baseline;
            bool const held = ratio >= keep_ratio;
            if (held)
            {
                ++held_streak_;
                missed_streak_ = 0;
            }
            else
            {
                ++missed_streak_;
                held_streak_ = 0;
            }
            RCLCPP_INFO(ctx_->logger(), "ConfirmGraspByScale: '%s' area ratio %.2f (keep >= %.2f) -> %s (%d/%d)",
                        label.c_str(), ratio, keep_ratio, held ? "held" : "missed",
                        held ? held_streak_ : missed_streak_, confirm_frames);
            if (held_streak_ >= confirm_frames)
            {
                RCLCPP_INFO(ctx_->logger(), "ConfirmGraspByScale: '%s' GRABBED (%d consecutive frames)", label.c_str(),
                            held_streak_);
                return BT::NodeStatus::SUCCESS;
            }
            if (missed_streak_ >= confirm_frames)
            {
                RCLCPP_INFO(ctx_->logger(), "ConfirmGraspByScale: '%s' MISSED (%d consecutive frames)", label.c_str(),
                            missed_streak_);
                return BT::NodeStatus::FAILURE;
            }
            return timed_out_fallback(label, absent_is_grabbed, timeout_msec);
        }
    }
    return timed_out_fallback(label, absent_is_grabbed, timeout_msec);  // unreachable; keeps -Wreturn-type happy
}

BT::NodeStatus ConfirmGraspByScale::timed_out_fallback(std::string const& label, bool absent_is_grabbed,
                                                       int timeout_msec)
{
    if ((ctx_->node->now() - start_time_).seconds() * 1000.0 <= timeout_msec)
    {
        return BT::NodeStatus::RUNNING;
    }
    // No full streak inside the bound (sparse frames, alternating categories,
    // or a dead stream): decide from the evidence we DID accumulate rather
    // than blindly applying the absent policy — two 'missed' frames at expiry
    // must not become a GRABBED verdict (that would be worse than the legacy
    // single-frame judgment).
    if (missed_streak_ > held_streak_)
    {
        RCLCPP_WARN(ctx_->logger(),
                    "ConfirmGraspByScale: timeout (%d ms) with missed-leaning evidence (%d vs %d) on "
                    "'%s'; assuming missed",
                    timeout_msec, missed_streak_, held_streak_, label.c_str());
        return BT::NodeStatus::FAILURE;
    }
    if (held_streak_ > missed_streak_)
    {
        RCLCPP_WARN(ctx_->logger(),
                    "ConfirmGraspByScale: timeout (%d ms) with held-leaning evidence (%d vs %d) on "
                    "'%s'; assuming grabbed",
                    timeout_msec, held_streak_, missed_streak_, label.c_str());
        return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_WARN(ctx_->logger(), "ConfirmGraspByScale: timeout (%d ms) with no usable evidence on '%s'; %s",
                timeout_msec, label.c_str(), absent_is_grabbed ? "assuming grabbed" : "assuming missed");
    return absent_is_grabbed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
