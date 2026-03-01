#include "hone_midpoint.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include <yolo_msgs/msg/detection_array.hpp>

HoneMidpoint::HoneMidpoint(std::string const& name, BT::NodeConfiguration const& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList HoneMidpoint::providedPorts()
{
    BT::PortsList ports;

    // Inputs
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));
    ports.insert(BT::InputPort<double>("min_conf", 0.25, "Min YOLO confidence"));
    ports.insert(BT::InputPort<double>("kp", 0.03, "Yaw P gain (cmd per pixel)"));
    ports.insert(BT::InputPort<double>("max_cmd", 0.6, "Max abs yaw cmd"));
    ports.insert(BT::InputPort<double>("tol_px", 10.0, "Centered tolerance in pixels"));
    ports.insert(BT::InputPort<int>("hold_ticks", 5, "Ticks within tol required for SUCCESS"));

    // Output
    ports.insert(BT::OutputPort<double>("yaw_cmd"));

    return ports;
}

BT::NodeStatus HoneMidpoint::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;

    centered_streak_ = 0;
    detected_midpoint_ = false;

    // reset tracking memory
    last_red_cx_ = 0.0;
    last_red_cy_ = 0.0;
    last_white_cx_ = 0.0;
    last_white_cy_ = 0.0;

    setOutput("yaw_cmd", 0.0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoneMidpoint::onRunning()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;

    double min_conf = 0.25, kp = 0.002, max_cmd = 0.6, tol_px = 10.0;
    int hold_ticks = 5;

    (void)getInput("min_conf", min_conf);
    (void)getInput("kp", kp);
    (void)getInput("max_cmd", max_cmd);
    (void)getInput("tol_px", tol_px);
    (void)getInput("hold_ticks", hold_ticks);

    // Read image width
    uint32_t W = 0;
    {
        std::scoped_lock lk(ctx_->img_mx);
        W = ctx_->img_width;
    }
    if (W == 0)
        return BT::NodeStatus::RUNNING;

    // Read latest detections
    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }
    if (!arr || arr->detections.empty())
        return BT::NodeStatus::RUNNING;

    auto area = [](yolo_msgs::msg::Detection const& d) -> double
    {
        double const w = std::max(0.0, d.bbox.size.x);
        double const h = std::max(0.0, d.bbox.size.y);
        return w * h;
    };

    /*
       - first time: lock onto best red+white (area*conf)
       - afterwards: track closest-to-last red+white to keep using the same poles
    */

    yolo_msgs::msg::Detection const* chosen_r = nullptr;
    yolo_msgs::msg::Detection const* chosen_w = nullptr;

    if (!detected_midpoint_)
    {
        // Pick largest-by-(area*conf) red & white in this frame
        double best_r_key = -1.0, best_w_key = -1.0;

        for (auto const& d : arr->detections)
        {
            if (d.score < min_conf)
                continue;

            double const k = area(d) * d.score;

            if (d.class_name == "red-pole" && k > best_r_key)
            {
                best_r_key = k;
                chosen_r = &d;
            }
            else if (d.class_name == "white-pole" && k > best_w_key)
            {
                best_w_key = k;
                chosen_w = &d;
            }
        }

        if (!chosen_r || !chosen_w)
            return BT::NodeStatus::RUNNING;

        // Lock initial positions
        last_red_cx_ = chosen_r->bbox.center.position.x;
        last_red_cy_ = chosen_r->bbox.center.position.y;
        last_white_cx_ = chosen_w->bbox.center.position.x;
        last_white_cy_ = chosen_w->bbox.center.position.y;

        detected_midpoint_ = true;
    }
    else
    {
        // Track "same" poles by nearest neighbor to last position (squared distance)
        auto dist2 = [](double ax, double ay, double bx, double by)
        {
            double dx = ax - bx;
            double dy = ay - by;
            return dx * dx + dy * dy;
        };

        double best_r_d2 = 1e30;
        double best_w_d2 = 1e30;

        for (auto const& d : arr->detections)
        {
            if (d.score < min_conf)
                continue;

            double const cx = d.bbox.center.position.x;
            double const cy = d.bbox.center.position.y;

            if (d.class_name == "red-pole")
            {
                double const d2 = dist2(cx, cy, last_red_cx_, last_red_cy_);
                if (d2 < best_r_d2)
                {
                    best_r_d2 = d2;
                    chosen_r = &d;
                }
            }
            else if (d.class_name == "white-pole")
            {
                double const d2 = dist2(cx, cy, last_white_cx_, last_white_cy_);
                if (d2 < best_w_d2)
                {
                    best_w_d2 = d2;
                    chosen_w = &d;
                }
            }
        }

        if (!chosen_r || !chosen_w)
            return BT::NodeStatus::RUNNING;

        // Update tracked positions
        last_red_cx_ = chosen_r->bbox.center.position.x;
        last_red_cy_ = chosen_r->bbox.center.position.y;
        last_white_cx_ = chosen_w->bbox.center.position.x;
        last_white_cy_ = chosen_w->bbox.center.position.y;
    }

    // Use the tracked/latest positions for control
    double const rx = last_red_cx_;
    double const wx = last_white_cx_;

    // P-control on midpoint pixel error
    double const mid_cx = 0.5 * (rx + wx);
    double const err_px = mid_cx - (double(W) / 2.0);

    double const yaw_cmd = clamp(-kp * err_px, -max_cmd, +max_cmd);
    setOutput("yaw_cmd", yaw_cmd);

    // Centered logic
    if (std::abs(err_px) <= tol_px)
        centered_streak_++;
    else
        centered_streak_ = 0;

    if (centered_streak_ >= hold_ticks)
    {
        setOutput("yaw_cmd", 0.0);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void HoneMidpoint::onHalted()
{
    centered_streak_ = 0;
    detected_midpoint_ = false;
    setOutput("yaw_cmd", 0.0);
}
