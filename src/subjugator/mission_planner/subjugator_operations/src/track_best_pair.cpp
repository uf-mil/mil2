#include "track_best_pair.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

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

BT::PortsList TrackBestPair::providedPorts()
{
    BT::PortsList ports;

    ports.insert(BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"));
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));
    ports.insert(BT::InputPort<int>("channel_side", 0, "-1=L, 0=unknown, 1=R"));

    ports.insert(BT::OutputPort<double>("best_qx"));
    ports.insert(BT::OutputPort<double>("best_qy"));
    ports.insert(BT::OutputPort<double>("best_qz"));
    ports.insert(BT::OutputPort<double>("best_qw"));

    return ports;
}

BT::NodeStatus TrackBestPair::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;

    best_pair_score_ = -1e18;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TrackBestPair::onRunning()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;

    double min_conf = 0.30;
    int channel_side = 0;
    (void)getInput("min_conf", min_conf);
    (void)getInput("channel_side", channel_side);

    // Snapshot latest detections (copy optional)
    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }
    if (!arr || arr->detections.empty())
        return BT::NodeStatus::RUNNING;

    auto area_of = [](yolo_msgs::msg::Detection const* d) -> double
    {
        double const w = std::max(0.0, d->bbox.size.x);
        double const h = std::max(0.0, d->bbox.size.y);
        return w * h;
    };

    auto key_of = [&](yolo_msgs::msg::Detection const* d) -> double
    {
        return area_of(d) * d->score;  // your desired heuristic
    };

    // Collect candidates
    std::vector<yolo_msgs::msg::Detection const*> reds, whites;
    reds.reserve(arr->detections.size());
    whites.reserve(arr->detections.size());

    for (auto const& d : arr->detections)
    {
        if (d.score < min_conf)
            continue;
        if (d.class_name == "red-pole")
            reds.push_back(&d);
        else if (d.class_name == "white-pole")
            whites.push_back(&d);
    }

    if (reds.empty() || whites.empty())
        return BT::NodeStatus::RUNNING;

    // Keep only top-K per color (more efficient + less clutter)
    auto keep_top_k = [&](std::vector<yolo_msgs::msg::Detection const*>& v, std::size_t K)
    {
        auto comparison = [&](yolo_msgs::msg::Detection const* a, yolo_msgs::msg::Detection const* b)
        { return key_of(a) > key_of(b); };
        if (v.size() > K)
        {
            std::nth_element(v.begin(), v.begin() + (K - 1), v.end(), comparison);
            v.resize(K);
        }
    };

    std::size_t const K = 3;  // tunable
    keep_top_k(reds, K);
    keep_top_k(whites, K);

    // Choose best pair among K^2 combos
    yolo_msgs::msg::Detection const* best_r = nullptr;
    yolo_msgs::msg::Detection const* best_w = nullptr;
    double best_score_frame = -1e18;

    for (auto r : reds)
    {
        for (auto w : whites)
        {
            if (channel_side < 0)
            {
                // Right channel: red is on our left
                if (r->bbox.center.position.x >= w->bbox.center.position.x)
                    continue;
            }
            else if (channel_side > 0)
            {
                // Left channel: red is on our right
                if (r->bbox.center.position.x <= w->bbox.center.position.x)
                    continue;
            }

            double const score = key_of(r) + key_of(w);
            if (score > best_score_frame)
            {
                best_score_frame = score;
                best_r = r;
                best_w = w;
            }
        }
    }

    if (!best_r || !best_w)
        return BT::NodeStatus::RUNNING;

    // If this frame is the best we've ever seen, snapshot orientation and output it
    if (best_score_frame > best_pair_score_)
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

        best_pair_score_ = best_score_frame;

        setOutput("best_qx", q.x);
        setOutput("best_qy", q.y);
        setOutput("best_qz", q.z);
        setOutput("best_qw", q.w);
    }

    return BT::NodeStatus::RUNNING;
}
