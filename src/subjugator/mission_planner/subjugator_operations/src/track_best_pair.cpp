#include "track_best_pair.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

namespace
{
std::uint64_t g_tick_count = 0;
bool g_have_last_tick = false;
std::chrono::steady_clock::time_point g_last_tick_tp{};

bool g_have_last_fingerprint = false;
std::uint64_t g_last_fingerprint = 0;

inline void hash_combine(std::uint64_t& seed, std::uint64_t value)
{
    seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
}

std::uint64_t quantize_to_u64(double x, double scale)
{
    long long const q = static_cast<long long>(std::llround(x * scale));
    return static_cast<std::uint64_t>(q);
}

std::uint64_t frame_fingerprint(yolo_msgs::msg::DetectionArray const& arr)
{
    std::uint64_t seed = 1469598103934665603ULL;

    hash_combine(seed, static_cast<std::uint64_t>(arr.detections.size()));

    for (auto const& d : arr.detections)
    {
        for (char c : d.class_name)
        {
            hash_combine(seed, static_cast<std::uint64_t>(static_cast<unsigned char>(c)));
        }

        hash_combine(seed, quantize_to_u64(d.score, 1000.0));
        hash_combine(seed, quantize_to_u64(d.bbox.center.position.x, 100.0));
        hash_combine(seed, quantize_to_u64(d.bbox.center.position.y, 100.0));
        hash_combine(seed, quantize_to_u64(d.bbox.size.x, 100.0));
        hash_combine(seed, quantize_to_u64(d.bbox.size.y, 100.0));
    }

    return seed;
}

void reset_tick_log()
{
    g_tick_count = 0;
    g_have_last_tick = false;
    g_have_last_fingerprint = false;
    g_last_fingerprint = 0;

    std::ofstream log("/tmp/track_best_pair_tick_log.csv", std::ios::out | std::ios::trunc);
    log << "tick,time_sec,dt_ms,hz,had_arr,num_detections,num_reds,num_whites,"
           "frame_fingerprint,frame_changed,best_score_frame,best_pair_score,"
           "red_area,white_area,red_cx,white_cx,updated_best\n";
}

void append_tick_log(bool had_arr, std::size_t num_detections, std::size_t num_reds, std::size_t num_whites,
                     std::uint64_t fingerprint, int frame_changed, double best_score_frame, double best_pair_score,
                     double red_area, double white_area, double red_cx, double white_cx, int updated_best)
{
    auto const now = std::chrono::steady_clock::now();

    double time_sec = std::chrono::duration<double>(now.time_since_epoch()).count();

    double dt_ms = -1.0;
    double hz = -1.0;

    if (g_have_last_tick)
    {
        dt_ms = std::chrono::duration<double, std::milli>(now - g_last_tick_tp).count();
        if (dt_ms > 1e-9)
        {
            hz = 1000.0 / dt_ms;
        }
    }

    g_last_tick_tp = now;
    g_have_last_tick = true;
    ++g_tick_count;

    std::ofstream log("/tmp/track_best_pair_tick_log.csv", std::ios::out | std::ios::app);
    log << std::fixed << std::setprecision(6) << g_tick_count << "," << time_sec << "," << dt_ms << "," << hz << ","
        << (had_arr ? 1 : 0) << "," << num_detections << "," << num_reds << "," << num_whites << "," << fingerprint
        << "," << frame_changed << "," << best_score_frame << "," << best_pair_score << "," << red_area << ","
        << white_area << "," << red_cx << "," << white_cx << "," << updated_best << "\n";
}
}  // namespace

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

    ports.insert(BT::InputPort<double>("min_conf", 0.15, "Minimum detection confidence"));
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

    // DEBUG: start a fresh tick log for this run
    reset_tick_log();

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

    // Debug fields with safe defaults
    std::size_t num_detections = arr ? arr->detections.size() : 0;
    std::size_t num_reds = 0;
    std::size_t num_whites = 0;
    std::uint64_t fingerprint = 0;
    int frame_changed = 0;
    double best_score_frame = -1.0;
    double red_area = -1.0;
    double white_area = -1.0;
    double red_cx = -1.0;
    double white_cx = -1.0;
    int updated_best = 0;

    if (arr)
    {
        fingerprint = frame_fingerprint(*arr);
        if (!g_have_last_fingerprint || fingerprint != g_last_fingerprint)
        {
            frame_changed = 1;
        }
        g_last_fingerprint = fingerprint;
        g_have_last_fingerprint = true;
    }

    if (!arr || arr->detections.empty())
    {
        append_tick_log(arr.has_value(), num_detections, num_reds, num_whites, fingerprint, frame_changed,
                        best_score_frame, best_pair_score_, red_area, white_area, red_cx, white_cx, updated_best);
        return BT::NodeStatus::RUNNING;
    }

    auto height_of = [](yolo_msgs::msg::Detection const* d) -> double { return std::max(0.0, d->bbox.size.y); };

    auto key_of = [&](yolo_msgs::msg::Detection const* d) -> double
    {
        return height_of(d);  // desired heuristic
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

    num_reds = reds.size();
    num_whites = whites.size();

    if (reds.empty() || whites.empty())
    {
        append_tick_log(true, num_detections, num_reds, num_whites, fingerprint, frame_changed, best_score_frame,
                        best_pair_score_, red_area, white_area, red_cx, white_cx, updated_best);
        return BT::NodeStatus::RUNNING;
    }

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

    std::size_t const K = 6;  // tunable
    keep_top_k(reds, K);
    keep_top_k(whites, K);

    // Choose best pair among K^2 combos
    yolo_msgs::msg::Detection const* best_r = nullptr;
    yolo_msgs::msg::Detection const* best_w = nullptr;
    best_score_frame = -1e18;

    for (auto r : reds)
    {
        for (auto w : whites)
        {
            double const rx = r->bbox.center.position.x;
            double const wx = w->bbox.center.position.x;
            double const ry = r->bbox.center.position.y;
            double const wy = w->bbox.center.position.y;

            (void)ry;
            (void)wy;

            if (channel_side < 0)
            {
                // red must be left of white
                if (rx >= wx)
                    continue;
            }
            else if (channel_side > 0)
            {
                // red must be right of white
                if (rx <= wx)
                    continue;
            }

            double const ar = height_of(r);
            double const aw = height_of(w);

            double const score = ar + aw;

            if (score > best_score_frame)
            {
                best_score_frame = score;
                best_r = r;
                best_w = w;
            }
        }
    }

    if (!best_r || !best_w)
    {
        append_tick_log(true, num_detections, num_reds, num_whites, fingerprint, frame_changed, best_score_frame,
                        best_pair_score_, red_area, white_area, red_cx, white_cx, updated_best);
        return BT::NodeStatus::RUNNING;
    }

    red_area = height_of(best_r);
    white_area = height_of(best_w);
    red_cx = best_r->bbox.center.position.x;
    white_cx = best_w->bbox.center.position.x;

    // If this frame is the best we've ever seen, snapshot orientation and output it
    if (best_score_frame > best_pair_score_)
    {
        geometry_msgs::msg::Pose current{};
        {
            std::scoped_lock lk(ctx_->odom_mx);
            if (!ctx_->latest_odom)
            {
                append_tick_log(true, num_detections, num_reds, num_whites, fingerprint, frame_changed,
                                best_score_frame, best_pair_score_, red_area, white_area, red_cx, white_cx,
                                updated_best);
                return BT::NodeStatus::RUNNING;
            }
            current = ctx_->latest_odom->pose.pose;
        }

        geometry_msgs::msg::Quaternion q = current.orientation;
        normalize_quat(q);

        best_pair_score_ = best_score_frame;
        updated_best = 1;

        setOutput("best_qx", q.x);
        setOutput("best_qy", q.y);
        setOutput("best_qz", q.z);
        setOutput("best_qw", q.w);
    }

    append_tick_log(true, num_detections, num_reds, num_whites, fingerprint, frame_changed, best_score_frame,
                    best_pair_score_, red_area, white_area, red_cx, white_cx, updated_best);

    return BT::NodeStatus::RUNNING;
}

void TrackBestPair::onHalted()
{
}
