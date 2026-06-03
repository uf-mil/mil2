#include "nav_channel_control.hpp"

#include <optional>
#include <vector>

#include <yolo_msgs/msg/detection_array.hpp>

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

    // Output magnitudes.
    ports.insert(BT::InputPort<double>("max_yaw_deg", 6.0, "Max abs yaw cmd per tick (deg)"));
    ports.insert(BT::InputPort<double>("max_strafe_m", 0.25, "Max abs strafe cmd per tick (m)"));

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

    // NOTE: centered_streak_ is intentionally NOT reset here so it can
    // accumulate across sequential Sequence iterations. It is only reset in
    // onHalted() (if the node is stopped while mid-run) or at construction.
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
    double max_yaw_deg = 6.0, max_strafe_m = 0.25;
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
    (void)getInput("max_yaw_deg", max_yaw_deg);
    (void)getInput("max_strafe_m", max_strafe_m);
    (void)getInput("tol_px", tol_px);
    (void)getInput("hold_ticks", hold_ticks);
    (void)getInput("right_channel", rc_in);

    uint32_t W = 0;
    {
        std::scoped_lock lk(ctx_->img_mx);
        W = ctx_->img_width;
    }
    if (W == 0)
    {
        // No image yet — emit a no-op command and let RelativeMove execute it.
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
        // No detections yet — emit a no-op command and continue.
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

    // "Nearness" of a pole = its bbox height in px (taller = physically closer).
    auto pole_h = [](yolo_msgs::msg::Detection const* d) -> double { return std::max(0.0, d->bbox.size.y); };

    // -------------------------------------------------------------------------
    // Channel-side resolution. Explicit input wins. Otherwise latch from the
    // NEAREST red/white pair (largest min-height) the first time we see one,
    // and infer the side from their left/right ordering.
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
        double const rx = nr->bbox.center.position.x;
        double const wx = nw->bbox.center.position.x;
        resolved_side_ = (rx < wx) ? +1 : -1;
    }
    setOutput("resolved_channel", resolved_side_);

    if (resolved_side_ == 0 || (reds.empty() && whites.empty()))
    {
        // Side not yet resolved / nothing to track — emit a no-op and continue.
        setOutput("yaw_cmd", 0.0);
        setOutput("y_cmd", 0.0);
        setOutput("mode_is_strafe", false);
        return BT::NodeStatus::SUCCESS;
    }

    // -------------------------------------------------------------------------
    // Expected normalized X for each pole on the resolved channel.
    // RIGHT (+1): red left-of-center, white right-of-center. LEFT (-1): mirrored.
    // -------------------------------------------------------------------------
    double const red_exp_norm = 0.5 - resolved_side_ * ideal_off;
    double const white_exp_norm = 0.5 + resolved_side_ * ideal_off;

    // Required left/right ordering for a valid gate on this channel, derived
    // straight from the expected positions so it can never drift out of sync
    // with the convention above:
    //   LEFT  (-1): red_exp > white_exp  -> red must sit RIGHT of white.
    //   RIGHT (+1): red_exp < white_exp  -> red must sit LEFT  of white.
    bool const need_red_right_of_white = (red_exp_norm > white_exp_norm);

    // -------------------------------------------------------------------------
    // Gate-aware selection.
    //
    // IMPORTANT: selection is based on NEARNESS (pole height), NOT on how close
    // a pole already is to its target position. Selecting by "closest to
    // expected" is a degenerate feedback loop — it always picks whatever pole
    // is already centred, so the measured error is tiny and the sub never
    // corrects. Instead we pick the nearest VALID gate (a red+white pair with
    // the correct ordering for this channel) and let the controller drive that
    // gate's real error to zero.
    //
    // Score = min(red_h, white_h): both poles of the pair must be large, so a
    // near-red + far-white cross-gate pairing loses to a genuine near gate.
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

    // Fallback: no valid gate pair visible (one pole off-screen/occluded, or
    // only wrong-ordering pairs exist). Steer toward the single NEAREST pole of
    // either colour so we keep making progress instead of freezing. Exactly one
    // of best_r / best_w is set here, so the single-pole (scale=2.0) path runs.
    if (!best_r && !best_w)
    {
        double best_near = -1.0;
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

    // -------------------------------------------------------------------------
    // Per-pole error and size-sigmoid weight.
    // -------------------------------------------------------------------------
    auto compute_pole = [&](yolo_msgs::msg::Detection const* d, double exp_norm, double& err_norm, double& size_w,
                            double& err_px_abs) -> bool
    {
        if (!d)
        {
            err_norm = 0.0;
            size_w = 0.0;
            err_px_abs = 0.0;
            return false;
        }
        double const x_norm = d->bbox.center.position.x / Wf;
        err_norm = x_norm - exp_norm;
        size_w = sigmoid_height(std::max(0.0, d->bbox.size.y), size_center, size_div);
        err_px_abs = std::abs(err_norm) * Wf;
        return true;
    };

    double red_e = 0.0, white_e = 0.0;
    double KrS = 0.0, KwS = 0.0;
    double red_err_px = 0.0, white_err_px = 0.0;

    bool const have_r = compute_pole(best_r, red_exp_norm, red_e, KrS, red_err_px);
    bool const have_w = compute_pole(best_w, white_exp_norm, white_e, KwS, white_err_px);

    double max_abs_e = 0.0;
    if (have_r)
        max_abs_e = std::max(max_abs_e, std::abs(red_e));
    if (have_w)
        max_abs_e = std::max(max_abs_e, std::abs(white_e));

    bool const strafe_mode = (max_abs_e > yaw_outer);
    bool const yaw_mode = !strafe_mode && (max_abs_e > yaw_dead);

    double yaw_cmd = 0.0;
    double y_cmd = 0.0;

    if (yaw_mode)
    {
        // Per-pole signed yaw triangle, weighted by closeness. Positive err
        // (pole drifted right of expected) -> yaw RIGHT to bring it back, which
        // is a negative yaw_deg in REP-103 (positive yaw = CCW = LEFT).
        double const red_y = signed_ramp(red_e, yaw_dead, yaw_outer) * KrS;
        double const white_y = signed_ramp(white_e, yaw_dead, yaw_outer) * KwS;
        int const pole_count = (int)have_r + (int)have_w;
        double const scale = (pole_count == 1) ? 2.0 : 1.0;
        double const raw = (red_y + white_y) * scale;
        yaw_cmd = clamp(-max_yaw_deg * raw, -max_yaw_deg, +max_yaw_deg);
    }
    else if (strafe_mode)
    {
        // Per-pole signed strafe triangle. Positive err -> sub should move
        // RIGHT (negative y in REP-103) so the pole drifts back to the left.
        double const red_s = signed_ramp(red_e, yaw_outer, strafe_sat) * KrS;
        double const white_s = signed_ramp(white_e, yaw_outer, strafe_sat) * KwS;
        int const pole_count = (int)have_r + (int)have_w;
        double const scale = (pole_count == 1) ? 2.0 : 1.0;
        double const raw = (red_s + white_s) * scale;
        y_cmd = clamp(-max_strafe_m * raw, -max_strafe_m, +max_strafe_m);
    }

    setOutput("yaw_cmd", yaw_cmd);
    setOutput("y_cmd", y_cmd);
    setOutput("mode_is_strafe", strafe_mode);

    double const max_err_px = std::max(have_r ? red_err_px : 0.0, have_w ? white_err_px : 0.0);
    if ((have_r || have_w) && max_err_px <= tol_px)
        centered_streak_++;
    else
        centered_streak_ = 0;

    if (centered_streak_ >= hold_ticks)
    {
        // Centered for hold_ticks consecutive ticks — signal the
        // KeepRunningUntilFailure wrapper to stop the loop.
        setOutput("yaw_cmd", 0.0);
        setOutput("y_cmd", 0.0);
        setOutput("mode_is_strafe", false);
        return BT::NodeStatus::FAILURE;
    }

    // Command computed — return SUCCESS so the Sequence can advance to
    // RelativeMove immediately this tick.
    return BT::NodeStatus::SUCCESS;
}

void NavChannelControl::onHalted()
{
    centered_streak_ = 0;
    setOutput("yaw_cmd", 0.0);
    setOutput("y_cmd", 0.0);
    setOutput("mode_is_strafe", false);
}
