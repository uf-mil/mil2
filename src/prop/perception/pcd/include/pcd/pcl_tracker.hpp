/**
 * @file pcl_tracker.hpp
 * @brief PclTracker — EKF-based multi-object tracker for lidar clusters.
 *
 * Subscribes to the "cluster_markers" topic published by PclClustering and
 * performs frame-to-frame data association so that each physical object
 * keeps the same integer track ID across frames.
 *
 * Algorithm (mirrors multi_object_tracking_lidar by Praveen Palanisamy,
 * ported to ROS 2 with a native C++ implementation):
 *
 *   1. For each incoming detection frame:
 *        a. Predict all existing tracks forward (constant-velocity EKF).
 *        b. Build a cost matrix: distance(predicted_centroid, detection).
 *        c. Greedy nearest-neighbour assignment (gated by max_association_dist).
 *        d. Update matched tracks; increment miss counter for unmatched tracks.
 *        e. Spawn new tracks for unmatched detections.
 *        f. Delete tracks whose miss counter exceeds max_missed_frames.
 *   2. Publish all confirmed tracks (hits >= min_hits) as a MarkerArray with
 *      stable .id fields and per-track colours.
 *
 * EKF state:  x = [px, py, vx, vy]^T  (2-D constant-velocity model)
 * Measurement: z = [px, py]^T          (centroid of bounding-box marker)
 */

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "pcd/pcd_constants.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace pcd
{

// ─────────────────────────────────────────────────────────────────────────────
// Minimal 4×4 / 2×4 matrix helpers (avoids pulling in Eigen)
// ─────────────────────────────────────────────────────────────────────────────
namespace ekf_math
{

using Mat44 = std::array<std::array<double, 4>, 4>;
using Mat24 = std::array<std::array<double, 4>, 2>;  // 2 rows × 4 cols
using Mat42 = std::array<std::array<double, 2>, 4>;  // 4 rows × 2 cols
using Mat22 = std::array<std::array<double, 2>, 2>;
using Vec4 = std::array<double, 4>;
using Vec2 = std::array<double, 2>;

inline Mat44 mat44_mul(Mat44 const& A, Mat44 const& B)
{
    Mat44 C{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

// A (4×4) × B^T (4×4) = C (4×4), but B given in row-major too
inline Mat44 mat44_mul_transpose(Mat44 const& A, Mat44 const& B)
{
    Mat44 C{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                C[i][j] += A[i][k] * B[j][k];  // B transposed
    return C;
}

inline Mat44 mat44_add(Mat44 const& A, Mat44 const& B)
{
    Mat44 C{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            C[i][j] = A[i][j] + B[i][j];
    return C;
}

// H (2×4) × P (4×4) = out (2×4)
inline Mat24 mat24_mul_44(Mat24 const& H, Mat44 const& P)
{
    Mat24 C{};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                C[i][j] += H[i][k] * P[k][j];
    return C;
}

// (2×4) × (4×2) = (2×2)
inline Mat22 mat24_mul_42(Mat24 const& A, Mat42 const& B)
{
    Mat22 C{};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 4; ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

// (4×2) × (2×2) = (4×2)
inline Mat42 mat42_mul_22(Mat42 const& A, Mat22 const& B)
{
    Mat42 C{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 2; ++j)
            for (int k = 0; k < 2; ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

// K (4×2) × H (2×4) = (4×4)
inline Mat44 mat42_mul_24(Mat42 const& A, Mat24 const& B)
{
    Mat44 C{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 2; ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

inline Mat22 mat22_add(Mat22 const& A, Mat22 const& B)
{
    return { { { A[0][0] + B[0][0], A[0][1] + B[0][1] }, { A[1][0] + B[1][0], A[1][1] + B[1][1] } } };
}

// 2×2 inverse
inline Mat22 mat22_inv(Mat22 const& A)
{
    double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    if (std::abs(det) < 1e-12)
        det = 1e-12;
    return { { { A[1][1] / det, -A[0][1] / det }, { -A[1][0] / det, A[0][0] / det } } };
}

// F×x (4×4 × 4-vec)
inline Vec4 mat44_mul_vec4(Mat44 const& A, Vec4 const& v)
{
    Vec4 out{};
    for (int i = 0; i < 4; ++i)
        for (int k = 0; k < 4; ++k)
            out[i] += A[i][k] * v[k];
    return out;
}

// H×x (2×4 × 4-vec)
inline Vec2 mat24_mul_vec4(Mat24 const& H, Vec4 const& v)
{
    Vec2 out{};
    for (int i = 0; i < 2; ++i)
        for (int k = 0; k < 4; ++k)
            out[i] += H[i][k] * v[k];
    return out;
}

// K (4×2) × vec2 → vec4
inline Vec4 mat42_mul_vec2(Mat42 const& K, Vec2 const& v)
{
    Vec4 out{};
    for (int i = 0; i < 4; ++i)
        for (int k = 0; k < 2; ++k)
            out[i] += K[i][k] * v[k];
    return out;
}

inline Mat44 mat44_identity()
{
    Mat44 I{};
    I[0][0] = I[1][1] = I[2][2] = I[3][3] = 1.0;
    return I;
}

}  // namespace ekf_math

// ─────────────────────────────────────────────────────────────────────────────
// Internal colour palette (one stable colour per track ID)
// ─────────────────────────────────────────────────────────────────────────────
namespace detail
{
struct TrackRgb
{
    float r, g, b;
};

inline TrackRgb track_color(int id)
{
    static constexpr TrackRgb kPalette[] = {
        { 0.90f, 0.10f, 0.29f }, { 0.24f, 0.71f, 0.29f }, { 1.00f, 0.88f, 0.10f }, { 0.00f, 0.51f, 0.78f },
        { 0.96f, 0.51f, 0.19f }, { 0.57f, 0.12f, 0.71f }, { 0.27f, 0.94f, 0.94f }, { 0.94f, 0.20f, 0.90f },
        { 0.82f, 0.96f, 0.24f }, { 0.98f, 0.75f, 0.83f }, { 0.00f, 0.50f, 0.50f }, { 0.86f, 0.75f, 1.00f },
    };
    return kPalette[static_cast<std::size_t>(id) % (sizeof(kPalette) / sizeof(kPalette[0]))];
}
}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// Track struct — one entry per tracked object
// ─────────────────────────────────────────────────────────────────────────────
struct Track
{
    int id{ -1 };
    int hits{ 0 };    ///< consecutive frames matched
    int missed{ 0 };  ///< consecutive frames without match

    // EKF state and covariance
    ekf_math::Vec4 x{};   ///< [px, py, vx, vy]
    ekf_math::Mat44 P{};  ///< 4×4 covariance

    // Last observed bounding-box scale (for marker rendering)
    double scale_x{ 0.5 };
    double scale_y{ 0.5 };
    double scale_z{ 0.5 };
    double pos_z{ 0.0 };  ///< Z passed through (not in EKF state)
};

// ─────────────────────────────────────────────────────────────────────────────
// PclTracker node
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class PclTracker
 * @brief ROS 2 node that subscribes to "cluster_markers" (MarkerArray) and
 *        publishes "tracked_markers" (MarkerArray) with stable track IDs.
 *
 * Subscribes:
 *   - "cluster_markers"  (visualization_msgs/MarkerArray)
 *
 * Publishes:
 *   - "tracked_markers"  (visualization_msgs/MarkerArray) — stable IDs
 */
class PclTracker : public rclcpp::Node, public PcdConstants
{
  public:
    PclTracker() : rclcpp::Node("pcl_tracker"), PcdConstants(this)
    {
        sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
            "cluster_markers", rclcpp::QoS(1), std::bind(&PclTracker::markers_cb, this, std::placeholders::_1));

        pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("tracked_markers", rclcpp::QoS(1));

        RCLCPP_INFO(get_logger(), "pcl_tracker started — assoc_dist=%.1f m  max_miss=%d  min_hits=%d",
                    max_association_dist_, max_missed_frames_, min_hits_);
    }

  private:
    // ── Helpers ───────────────────────────────────────────────────────────────

    /// Build a new track from a MarkerArray detection (a single CUBE marker).
    Track make_track(visualization_msgs::msg::Marker const& m)
    {
        Track t;
        t.id = next_id_++;
        t.hits = 1;
        t.missed = 0;

        // EKF initial state: position from marker centroid, zero velocity.
        t.x = { m.pose.position.x, m.pose.position.y, 0.0, 0.0 };

        // Initial covariance — large positional uncertainty, zero velocity.
        t.P = {};
        t.P[0][0] = 5.0;   // px variance
        t.P[1][1] = 5.0;   // py variance
        t.P[2][2] = 10.0;  // vx variance
        t.P[3][3] = 10.0;  // vy variance

        t.scale_x = m.scale.x;
        t.scale_y = m.scale.y;
        t.scale_z = m.scale.z;
        t.pos_z = m.pose.position.z;
        return t;
    }

    /// EKF predict step.
    void ekf_predict(Track& t, double dt)
    {
        using namespace ekf_math;

        // Transition matrix F (constant velocity)
        Mat44 F = mat44_identity();
        F[0][2] = dt;
        F[1][3] = dt;

        // x = F * x
        t.x = mat44_mul_vec4(F, t.x);

        // P = F * P * F^T + Q
        double const q_pos = 0.5;  // process noise — position
        double const q_vel = 2.0;  // process noise — velocity

        Mat44 Q{};
        Q[0][0] = q_pos * dt * dt;
        Q[1][1] = q_pos * dt * dt;
        Q[2][2] = q_vel;
        Q[3][3] = q_vel;

        t.P = mat44_add(mat44_mul_transpose(mat44_mul(F, t.P), F), Q);
    }

    /// EKF update step with measurement z = [px, py].
    void ekf_update(Track& t, double meas_x, double meas_y)
    {
        using namespace ekf_math;

        // H (2×4): maps state to measurement
        Mat24 H{};
        H[0][0] = 1.0;
        H[1][1] = 1.0;

        // H^T (4×2)
        Mat42 Ht{};
        Ht[0][0] = 1.0;
        Ht[1][1] = 1.0;

        // Measurement noise R
        Mat22 R{};
        R[0][0] = 0.5;
        R[1][1] = 0.5;

        // Innovation covariance S = H * P * H^T + R
        Mat24 HP = mat24_mul_44(H, t.P);
        Mat22 S = mat22_add(mat24_mul_42(HP, Ht), R);
        Mat22 S_inv = mat22_inv(S);

        // Kalman gain K = P * H^T * S_inv  (4×2)
        Mat42 PHt_real{};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 4; ++k)
                    PHt_real[i][j] += t.P[i][k] * Ht[k][j];
        Mat42 K = mat42_mul_22(PHt_real, S_inv);

        // Innovation y = z - H*x
        Vec2 z = { meas_x, meas_y };
        Vec2 Hx = mat24_mul_vec4(H, t.x);
        Vec2 innov = { z[0] - Hx[0], z[1] - Hx[1] };

        // State update x = x + K * y
        Vec4 Ky = mat42_mul_vec2(K, innov);
        for (int i = 0; i < 4; ++i)
            t.x[i] += Ky[i];

        // Covariance update P = (I - K*H) * P
        Mat44 KH = mat42_mul_24(K, H);
        Mat44 I = mat44_identity();
        Mat44 IKH{};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                IKH[i][j] = I[i][j] - KH[i][j];
        t.P = mat44_mul(IKH, t.P);
    }

    // ── Callback ──────────────────────────────────────────────────────────────

    void markers_cb(visualization_msgs::msg::MarkerArray::ConstSharedPtr const msg)
    {
        // Collect only CUBE (bounding-box) markers; ignore the DELETEALL sentinel.
        std::vector<visualization_msgs::msg::Marker const*> detections;
        for (auto const& m : msg->markers)
        {
            if (m.type == visualization_msgs::msg::Marker::CUBE && m.action == visualization_msgs::msg::Marker::ADD)
            {
                detections.push_back(&m);
            }
        }

        // Compute dt from stamp
        rclcpp::Time now = msg->markers.empty() ? this->now() : rclcpp::Time(msg->markers.back().header.stamp);
        double dt = 0.1;  // default fallback
        if (last_stamp_.nanoseconds() > 0)
        {
            double d = (now - last_stamp_).seconds();
            if (d > 0.001 && d < 2.0)
                dt = d;
        }
        last_stamp_ = now;
        std_msgs::msg::Header header = msg->markers.empty() ? std_msgs::msg::Header{} : msg->markers.back().header;

        // ── 1. Predict all tracks ─────────────────────────────────────────────
        for (auto& t : tracks_)
            ekf_predict(t, dt);

        // ── 2. Build cost matrix and assign ──────────────────────────────────
        std::size_t const nT = tracks_.size();
        std::size_t const nD = detections.size();

        // cost[i][j] = Euclidean distance between track i prediction and detection j
        std::vector<std::vector<double>> cost(nT, std::vector<double>(nD, 0.0));
        for (std::size_t i = 0; i < nT; ++i)
        {
            for (std::size_t j = 0; j < nD; ++j)
            {
                double dx = tracks_[i].x[0] - detections[j]->pose.position.x;
                double dy = tracks_[i].x[1] - detections[j]->pose.position.y;
                cost[i][j] = std::sqrt(dx * dx + dy * dy);
            }
        }

        // Greedy nearest-neighbour assignment (sufficient for low-density marine scene)
        std::vector<int> track_to_det(nT, -1);  // which detection matched each track
        std::vector<bool> det_used(nD, false);

        for (std::size_t i = 0; i < nT; ++i)
        {
            double best_cost = max_association_dist_;
            int best_j = -1;
            for (std::size_t j = 0; j < nD; ++j)
            {
                if (!det_used[j] && cost[i][j] < best_cost)
                {
                    best_cost = cost[i][j];
                    best_j = static_cast<int>(j);
                }
            }
            if (best_j >= 0)
            {
                track_to_det[i] = best_j;
                det_used[best_j] = true;
            }
        }

        // ── 3. Update matched / increment misses ──────────────────────────────
        for (std::size_t i = 0; i < nT; ++i)
        {
            int j = track_to_det[i];
            if (j >= 0)
            {
                auto const* d = detections[static_cast<std::size_t>(j)];
                ekf_update(tracks_[i], d->pose.position.x, d->pose.position.y);
                tracks_[i].hits++;
                tracks_[i].missed = 0;
                // Update bounding-box scale from latest detection
                tracks_[i].scale_x = d->scale.x;
                tracks_[i].scale_y = d->scale.y;
                tracks_[i].scale_z = d->scale.z;
                tracks_[i].pos_z = d->pose.position.z;
            }
            else
            {
                tracks_[i].missed++;
            }
        }

        // ── 4. Spawn new tracks for unmatched detections ──────────────────────
        for (std::size_t j = 0; j < nD; ++j)
        {
            if (!det_used[j])
                tracks_.push_back(make_track(*detections[j]));
        }

        // ── 5. Prune stale tracks ─────────────────────────────────────────────
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                     [this](Track const& t) { return t.missed > max_missed_frames_; }),
                      tracks_.end());

        // ── 6. Publish confirmed tracks ───────────────────────────────────────
        publish_tracks(header);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Tracks: %zu total  |  detections: %zu  |  published: %zu", tracks_.size(), nD,
                             confirmed_count_);
    }

    // ── Publishing ────────────────────────────────────────────────────────────

    void publish_tracks(std_msgs::msg::Header const& header)
    {
        visualization_msgs::msg::MarkerArray out;

        // DELETEALL sentinel to clear stale markers in RViz
        visualization_msgs::msg::Marker clear;
        clear.header = header;
        clear.ns = "tracked_objects";
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        out.markers.push_back(clear);

        confirmed_count_ = 0;
        for (auto const& t : tracks_)
        {
            if (t.hits < min_hits_)
                continue;  // not yet confirmed
            ++confirmed_count_;

            detail::TrackRgb col = detail::track_color(t.id);

            visualization_msgs::msg::Marker box;
            box.header = header;
            box.ns = "tracked_objects";
            box.id = t.id;
            box.type = visualization_msgs::msg::Marker::CUBE;
            box.action = visualization_msgs::msg::Marker::ADD;
            box.pose.position.x = t.x[0];   // EKF-filtered X
            box.pose.position.y = t.x[1];   // EKF-filtered Y
            box.pose.position.z = t.pos_z;  // pass-through Z
            box.pose.orientation.w = 1.0;
            box.scale.x = std::max(0.05, t.scale_x);
            box.scale.y = std::max(0.05, t.scale_y);
            box.scale.z = std::max(0.05, t.scale_z);
            box.color.r = col.r;
            box.color.g = col.g;
            box.color.b = col.b;
            box.color.a = 0.45f;
            box.lifetime = rclcpp::Duration(0, 0);
            out.markers.push_back(box);
        }

        pub_->publish(out);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

    std::vector<Track> tracks_;
    int next_id_{ 0 };
    std::size_t confirmed_count_{ 0 };
    rclcpp::Time last_stamp_{ 0, 0, RCL_ROS_TIME };
};

}  // namespace pcd
