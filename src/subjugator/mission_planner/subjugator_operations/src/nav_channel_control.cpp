#include "nav_channel_control.hpp"

#include <optional>
#include <vector>

#include <yolo_msgs/msg/detection_array.hpp>

REGISTER(NavChannelControl)

NavChannelControl::NavChannelControl(std::string const& name, BT::NodeConfiguration const& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList NavChannelControl::providedPorts()
{
    BT::PortsList ports;

    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));
    ports.insert(BT::InputPort<int>("right_channel", 0, "+1 RIGHT, -1 LEFT, 0 AUTO (latch from first confident pair)"));

    ports.insert(BT::InputPort<double>("min_conf", 0.15, "Min YOLO confidence"));

    // Pole-size sigmoid (input: bbox height in px).
    ports.insert(BT::InputPort<double>("size_center_px", 80.0, "Pole height (px) where the size weight = 0.5"));
    ports.insert(BT::InputPort<double>("size_div_px", 20.0, "Sigmoid steepness divisor (smaller = steeper)"));

    // Geometry / triangles.
    ports.insert(BT::InputPort<double>("ideal_offset_norm", 0.20,
                                       "How far from screen center each pole should sit (norm width)"));
    ports.insert(BT::InputPort<double>("yaw_deadzone_norm", 0.05, "Below this |err| (norm width), no yaw"));
    ports.insert(BT::InputPort<double>("yaw_outer_norm", 0.22, "|err| at which we switch from yaw to strafe"));
    ports.insert(BT::InputPort<double>("strafe_saturate_norm", 0.42, "|err| where strafe magnitude saturates"));

    // Asymmetric (outward) error handling. A pole drifting further in its
    // DESIRED outward direction (white going left on a left channel, red going
    // right) is good — it's exiting the frame as we pass through — so its error
    // is hard-zeroed and produces no correction. The exception is a very large
    // (close) pole, where an inward-ish position is a real clip risk: above
    // outward_keep_px the outward discount is disabled and the error is honored.
    ports.insert(BT::InputPort<double>("outward_keep_px", 250.0,
                                       "Pole height (px) above which outward drift is still corrected (clip guard)"));

    // Output magnitudes.
    ports.insert(BT::InputPort<double>("max_yaw_deg", 6.0, "Max abs yaw cmd per tick (deg)"));
    ports.insert(BT::InputPort<double>("max_strafe_m", 0.25, "Max abs strafe cmd per tick (m)"));

    // Command smoothing: EMA blend factor in [0,1]. 1.0 = no smoothing (raw
    // command each tick), lower = more damping of tick-to-tick reversals.
    ports.insert(BT::InputPort<double>("cmd_alpha", 0.7, "EMA blend factor for output smoothing (0=frozen,1=raw)"));

    // SUCCESS criterion.
    ports.insert(BT::InputPort<double>("tol_px", 15.0, "Per-pole pixel tolerance for SUCCESS"));
    ports.insert(BT::InputPort<int>("hold_ticks", 5, "Consecutive centered ticks for SUCCESS"));

    // Outputs.
    ports.insert(BT::OutputPort<double>("yaw_cmd", "Yaw step (deg) for this tick"));
    ports.insert(BT::OutputPort<double>("y_cmd", "Strafe step (m, +y = left) for this tick"));
    ports.insert(BT::OutputPort<bool>("mode_is_strafe", "True if this tick emitted a strafe step"));
    ports.insert(BT::OutputPort<int>("resolved_channel", "Channel side actually used (+1/-1/0)"));

    return ports;
}

BT::NodeStatus NavChannelControl::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;

    int rc_in = 0;
    (void)getInput("right_channel", rc_in);
    resolved_side_ = (rc_in == 1 || rc_in == -1) ? rc_in : 0;

    // Fresh start for each alignment phase: don't carry streak or smoothed
    // command state over from a previously-passed gate.
    centered_streak_ = 0;
    smoothed_yaw_ = 0.0;
    smoothed_y_ = 0.0;

    setOutput("yaw_cmd", 0.0);
    setOutput("y_cmd", 0.0);
    setOutput("mode_is_strafe", false);
    setOutput("resolved_channel", resolved_side_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavChannelControl::onRunning()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;

    double min_conf = 0.15;
    double size_center = 80.0, size_div = 20.0;
    double ideal_off = 0.20;
    double yaw_dead = 0.05, yaw_outer = 0.22, strafe_sat = 0.42;
    double outward_keep = 250.0;
    double max_yaw_deg = 6.0, max_strafe_m = 0.25;
    double cmd_alpha = 0.7;
    double tol_px = 15.0;
    int hold_ticks = 5;
    int rc_in = 0;

    (void)getInput("min_conf", min_conf);
    (void)getInput("size_center_px", size_center);
    (void)getInput("size_div_px", size_div);
    (void)getInput("ideal_offset_norm", ideal_off);
    (void)getInput("yaw_deadzone_norm", yaw_dead);
    (void)getInput("yaw_outer_norm", yaw_outer);
    (void)getInput("strafe_saturate_norm", strafe_sat);
    (void)getInput("outward_keep_px", outward_keep);
    (void)getInput("max_yaw_deg", max_yaw_deg);
    (void)getInput("max_strafe_m", max_strafe_m);
    (void)getInput("cmd_alpha", cmd_alpha);
    (void)getInput("tol_px", tol_px);
    (void)getInput("hold_ticks", hold_ticks);
    (void)getInput("right_channel", rc_in);

    cmd_alpha = std::max(0.0, std::min(1.0, cmd_alpha));

    uint32_t W = 0;
    {
        std::scoped_lock lk(ctx_->img_mx);
        W = ctx_->img_width;
    }
    if (W == 0)
    {
        setOutput("yaw_cmd", 0.0);
        setOutput("y_cmd", 0.0);
        setOutput("mode_is_strafe", false);
        return BT::NodeStatus::SUCCESS;
    }
    double const Wf = static_cast<double>(W);

    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }
    if (!arr || arr->detections.empty())
    {
        setOutput("yaw_cmd", 0.0);
        setOutput("y_cmd", 0.0);
        setOutput("mode_is_strafe", false);
        return BT::NodeStatus::SUCCESS;
    }

    // -------------------------------------------------------------------------
    // Collect all confident poles of each colour.
    // -------------------------------------------------------------------------
    std::vector<yolo_msgs::msg::Detection const*> reds, whites;
    for (auto const& d : arr->detections)
    {
        if (d.score < min_conf)
            continue;
        if (d.class_name == "red-pole")
            reds.push_back(&d);
        else if (d.class_name == "white-pole")
            whites.push_back(&d);
    }

    // Nearness proxy: bbox height in px (taller = physically closer).
    auto pole_h = [](yolo_msgs::msg::Detection const* d) -> double { return std::max(0.0, d->bbox.size.y); };

    // -------------------------------------------------------------------------
    // Channel-side resolution.
    // -------------------------------------------------------------------------
    if (rc_in == 1 || rc_in == -1)
    {
        resolved_side_ = rc_in;
    }
    else if (resolved_side_ == 0 && !reds.empty() && !whites.empty())
    {
        yolo_msgs::msg::Detection const* nr = nullptr;
        yolo_msgs::msg::Detection const* nw = nullptr;
        double best_near = -1.0;
        for (auto* r : reds)
            for (auto* w : whites)
            {
                double const near = std::min(pole_h(r), pole_h(w));
                if (near > best_near)
                {
                    best_near = near;
                    nr = r;
                    nw = w;
                }
            }
        resolved_side_ = (nr->bbox.center.position.x < nw->bbox.center.position.x) ? +1 : -1;
    }
    setOutput("resolved_channel", resolved_side_);

    if (resolved_side_ == 0 || (reds.empty() && whites.empty()))
    {
        setOutput("yaw_cmd", 0.0);
        setOutput("y_cmd", 0.0);
        setOutput("mode_is_strafe", false);
        return BT::NodeStatus::SUCCESS;
    }

    // -------------------------------------------------------------------------
    // Expected normalized X for each pole on the resolved channel.
    //   RIGHT (+1): red left  (0.30), white right (0.70)
    //   LEFT  (-1): red right (0.70), white left  (0.30)
    // -------------------------------------------------------------------------
    double const red_exp_norm = 0.5 - resolved_side_ * ideal_off;
    double const white_exp_norm = 0.5 + resolved_side_ * ideal_off;

    bool const need_red_right_of_white = (red_exp_norm > white_exp_norm);

    // Which screen half each colour belongs on. Also defines the "outward"
    // direction: a pole expected on the right drifts outward by going further
    // right (positive error); one expected on the left drifts outward by going
    // further left (negative error).
    bool const red_expected_right = (red_exp_norm > 0.5);
    bool const white_expected_right = (white_exp_norm > 0.5);

    // -------------------------------------------------------------------------
    // Gate-aware pair selection. Score = min(red_h, white_h); ordering
    // constraint rejects pairs whose geometry contradicts the channel.
    // -------------------------------------------------------------------------
    yolo_msgs::msg::Detection const* best_r = nullptr;
    yolo_msgs::msg::Detection const* best_w = nullptr;
    double best_pair_near = -1.0;

    for (auto* r : reds)
        for (auto* w : whites)
        {
            double const rx = r->bbox.center.position.x;
            double const wx = w->bbox.center.position.x;
            bool const ordering_ok = need_red_right_of_white ? (rx > wx) : (rx < wx);
            if (!ordering_ok)
                continue;
            double const near = std::min(pole_h(r), pole_h(w));
            if (near > best_pair_near)
            {
                best_pair_near = near;
                best_r = r;
                best_w = w;
            }
        }

    // -------------------------------------------------------------------------
    // Single-pole fallback with screen-side filter (rejects wrong-gate poles
    // on the wrong half), then a last-resort nearest-any pick so a badly
    // drifted sub still gets some correction rather than freezing.
    // -------------------------------------------------------------------------
    if (!best_r && !best_w)
    {
        double best_near = -1.0;

        for (auto* r : reds)
        {
            bool const right_half = (r->bbox.center.position.x / Wf > 0.5);
            if (right_half != red_expected_right)
                continue;
            if (pole_h(r) > best_near)
            {
                best_near = pole_h(r);
                best_r = r;
                best_w = nullptr;
            }
        }
        for (auto* w : whites)
        {
            bool const right_half = (w->bbox.center.position.x / Wf > 0.5);
            if (right_half != white_expected_right)
                continue;
            if (pole_h(w) > best_near)
            {
                best_near = pole_h(w);
                best_w = w;
                best_r = nullptr;
            }
        }

        if (!best_r && !best_w)
        {
            for (auto* r : reds)
                if (pole_h(r) > best_near)
                {
                    best_near = pole_h(r);
                    best_r = r;
                    best_w = nullptr;
                }
            for (auto* w : whites)
                if (pole_h(w) > best_near)
                {
                    best_near = pole_h(w);
                    best_w = w;
                    best_r = nullptr;
                }
        }
    }

    // -------------------------------------------------------------------------
    // Per-pole error, size weight, and ASYMMETRIC outward-error cutoff.
    //
    // A pole whose error points further in its desired outward direction is
    // doing exactly what we want as we pass through the gate, so we hard-zero
    // that error: no correction, and it reads as "centered" for the SUCCESS
    // check. The cutoff is disabled once the pole is very large (>= outward_keep
    // px), where an inward-ish close pole is a genuine clip risk worth honoring.
    // -------------------------------------------------------------------------
    auto compute_pole = [&](yolo_msgs::msg::Detection const* d, double exp_norm, bool expected_right, double& err_norm,
                            double& size_w, double& err_px_abs) -> bool
    {
        if (!d)
        {
            err_norm = 0.0;
            size_w = 0.0;
            err_px_abs = 0.0;
            return false;
        }

        double const h = std::max(0.0, d->bbox.size.y);
        double const x_norm = d->bbox.center.position.x / Wf;

        err_norm = x_norm - exp_norm;
        size_w = sigmoid_height(h, size_center, size_div);

        // outward == error is in the pole's desired away-from-center direction.
        bool const outward = (err_norm > 0.0) == expected_right;
        bool const pole_big = (h >= outward_keep);
        if (outward)
            err_norm = 0.0;  // discount: this drift is good, command nothing.

        err_px_abs = std::abs(err_norm) * Wf;
        return true;
    };

    double red_e = 0.0, white_e = 0.0;
    double KrS = 0.0, KwS = 0.0;
    double red_err_px = 0.0, white_err_px = 0.0;

    bool const have_r = compute_pole(best_r, red_exp_norm, red_expected_right, red_e, KrS, red_err_px);
    bool const have_w = compute_pole(best_w, white_exp_norm, white_expected_right, white_e, KwS, white_err_px);

    double max_abs_e = 0.0;
    if (have_r)
        max_abs_e = std::max(max_abs_e, std::abs(red_e));
    if (have_w)
        max_abs_e = std::max(max_abs_e, std::abs(white_e));

    bool const strafe_mode = (max_abs_e > yaw_outer);
    bool const yaw_mode = !strafe_mode && (max_abs_e > yaw_dead);

    double raw_yaw = 0.0;
    double raw_y = 0.0;

    if (yaw_mode)
    {
        double const red_y = signed_ramp(red_e, yaw_dead, yaw_outer) * KrS;
        double const white_y = signed_ramp(white_e, yaw_dead, yaw_outer) * KwS;
        int const n = (int)have_r + (int)have_w;
        double const scale = (n == 1) ? 2.0 : 1.0;
        raw_yaw = clamp(-max_yaw_deg * (red_y + white_y) * scale, -max_yaw_deg, +max_yaw_deg);
    }
    else if (strafe_mode)
    {
        double const red_s = signed_ramp(red_e, yaw_outer, strafe_sat) * KrS;
        double const white_s = signed_ramp(white_e, yaw_outer, strafe_sat) * KwS;
        int const n = (int)have_r + (int)have_w;
        double const scale = (n == 1) ? 2.0 : 1.0;
        raw_y = clamp(-max_strafe_m * (red_s + white_s) * scale, -max_strafe_m, +max_strafe_m);
    }

    // EMA smoothing to damp tick-to-tick reversals.
    if (strafe_mode || yaw_mode)
    {
        smoothed_yaw_ = cmd_alpha * raw_yaw + (1.0 - cmd_alpha) * smoothed_yaw_;
        smoothed_y_ = cmd_alpha * raw_y + (1.0 - cmd_alpha) * smoothed_y_;
    }
    else
    {
        smoothed_yaw_ *= (1.0 - cmd_alpha);
        smoothed_y_ *= (1.0 - cmd_alpha);
    }

    setOutput("yaw_cmd", smoothed_yaw_);
    setOutput("y_cmd", smoothed_y_);
    setOutput("mode_is_strafe", strafe_mode);

    // SUCCESS check uses the DISCOUNTED error: poles drifting outward read as
    // centered, so two outward poles (we're passing through) build the streak
    // and the loop declares the gate done.
    double const max_err_px = std::max(have_r ? red_err_px : 0.0, have_w ? white_err_px : 0.0);
    if ((have_r || have_w) && max_err_px <= tol_px)
        centered_streak_++;
    else
        centered_streak_ = 0;

    if (centered_streak_ >= hold_ticks)
    {
        setOutput("yaw_cmd", 0.0);
        setOutput("y_cmd", 0.0);
        setOutput("mode_is_strafe", false);
        return BT::NodeStatus::FAILURE;  // aligned / passing through — stop the loop
    }

    return BT::NodeStatus::SUCCESS;  // command computed — advance to RelativeMove
}

void NavChannelControl::onHalted()
{
    centered_streak_ = 0;
    smoothed_yaw_ = 0.0;
    smoothed_y_ = 0.0;
    setOutput("yaw_cmd", 0.0);
    setOutput("y_cmd", 0.0);
    setOutput("mode_is_strafe", false);
}
