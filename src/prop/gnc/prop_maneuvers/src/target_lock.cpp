#include "prop_maneuvers/target_lock.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace prop_maneuvers
{
namespace
{
/// How long to wait for the transform at a scan's own stamp before taking the degraded latest-transform
/// path; a small fraction of the clustering's ~3 Hz frame.
constexpr auto kTransformWait = std::chrono::milliseconds(50);

/// Keep an implausible cluster from sizing a standoff or clearance: a merged blob's half-width is not
/// the object's radius.
Blob sane(Blob blob, double limit, rclcpp::Node *node)
{
    if (blob.radius > limit)
    {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "clustering reported a %.1f m radius; clamping to %.1f m", blob.radius, limit);
        blob.radius = limit;
    }
    return blob;
}
}  // namespace

TargetLock::TargetLock(rclcpp::Node *node, Constants const &settings)
  : node_(node), settings_(settings), locked_at_(node->now())
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    markers_subscription_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        "cluster_markers", rclcpp::QoS(1),
        [this](visualization_msgs::msg::MarkerArray::SharedPtr const msg) { latest_ = *msg; });
}

namespace
{
/// The blobs that could plausibly be the object we are tracking. Merged clusters are dropped here only;
/// they are still avoided, but adopting one as the target hands the approach a phantom.
std::vector<Blob> real_only(std::vector<Blob> blobs)
{
    blobs.erase(std::remove_if(blobs.begin(), blobs.end(), [](Blob const &b) { return b.merged; }), blobs.end());
    return blobs;
}
}  // namespace

std::vector<Blob> TargetLock::blobs() const
{
    std::vector<Blob> out;

    for (auto const &marker : latest_.markers)
    {
        // The clustering sends a DELETEALL marker first, to clear the previous
        // frame's boxes. It carries no position and must be skipped.
        if (marker.action != visualization_msgs::msg::Marker::ADD)
        {
            continue;
        }

        geometry_msgs::msg::PointStamped in;
        in.header = marker.header;
        in.point = marker.pose.position;

        geometry_msgs::msg::PointStamped out_point;
        try
        {
            // Transform at the marker's own stamp, not the latest: pairing old lidar data with a newer pose swings
            // a static object by range times the yaw in between (a buoy 5.2 m away appeared to jump 1.93 m).
            // tf2 interpolates, so this is cheap. Wait briefly for it: the scan is stamped before the pose chain
            // publishes, so the right transform often does not exist yet. The wait stays under the clustering period
            // so a truly absent transform fails fast.
            out_point = tf_buffer_->transform(in, "map", kTransformWait);
        }
        catch (tf2::TransformException const &error)
        {
            // Fall back to the latest transform rather than going blind. This reintroduces the swing above, so it is
            // degraded.
            try
            {
                geometry_msgs::msg::PointStamped latest = in;
                latest.header.stamp = rclcpp::Time(0);
                out_point = tf_buffer_->transform(latest, "map");
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                     "no transform at the scan's own time (%s); falling back to the latest one, "
                                     "which smears object positions while turning",
                                     error.what());
            }
            catch (tf2::TransformException const &fallback_error)
            {
                // Skip this marker, not the whole frame. An empty list is read elsewhere as "we have stopped seeing"
                // (Phase::BackingOff aborts a reverse on it), so one bad marker must not manufacture one.
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                     "cannot place a blob on the map: %s (skipping it)", fallback_error.what());
                continue;
            }
        }

        // The clustering publishes a box; treat the larger ground dimension as a diameter.
        // Clamped here for every consumer: merged clusters over open water gave radii of 3-9 m that inflated
        // standoffs and blocked the reverse strip. Clamp rather than discard, since something may be there;
        // anything on this course is about 0.46 m across, so max_object_radius is already generous.
        double const width = std::max(marker.scale.x, marker.scale.y);
        double radius = width / 2.0;
        bool merged = false;
        if (radius > settings_.max_object_radius_)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "clustering reported a %.1f m radius blob; clamping to %.1f m -- probably merged "
                                 "returns rather than one object",
                                 radius, settings_.max_object_radius_);
            radius = settings_.max_object_radius_;
            merged = true;
        }
        out.push_back(Blob{ Point{ out_point.point.x, out_point.point.y }, radius, merged });
    }

    return out;
}

bool TargetLock::acquire_near(Point const &hint)
{
    // real_only, as in refresh(): a merged blob must never be the target. This is the acquisition path
    // the nodes take by default, so leaving it raw left the most-used path open to a phantom.
    auto const all = real_only(blobs());
    // The hint must land within match_radius of the real buoy; the larger acquire_max_range would trip the ambiguity
    // check.
    Match const match = match_nearest(all, hint, settings_.match_radius_, settings_.ambiguous_margin_);

    if (!match.ok)
    {
        why_ = describe(match.failure);
        RCLCPP_WARN(node_->get_logger(), "could not lock on near (%.1f, %.1f): %s", hint.x, hint.y, why_.c_str());
        return false;
    }

    locked_ = sane(match.blob, settings_.max_object_radius_, node_);
    locked_at_ = node_->now();
    why_.clear();
    RCLCPP_INFO(node_->get_logger(), "locked on at (%.1f, %.1f), radius %.2f m", locked_->centre.x, locked_->centre.y,
                locked_->radius);
    return true;
}

bool TargetLock::acquire_in_front(Point const &boat, double boat_direction)
{
    std::vector<Blob> candidates;
    for (auto const &blob : real_only(blobs()))
    {
        double const range = distance(boat, blob.centre);
        if (range > settings_.acquire_max_range_)
        {
            continue;
        }
        double const relative = wrap_angle(bearing(boat, blob.centre) - boat_direction);
        if (std::abs(relative) <= settings_.acquire_cone_)
        {
            candidates.push_back(blob);
        }
    }

    if (candidates.empty())
    {
        why_ = "nothing in front of the boat";
        RCLCPP_WARN(node_->get_logger(), "could not lock on: %s", why_.c_str());
        return false;
    }

    // Nearest in the cone wins. Ambiguity is not checked: there is no prediction to compare against, unlike
    // acquire_near and refresh. Two buoys at similar range resolve silently to the nearer one, and this is
    // the only acquisition path meant for autonomous use on the real boat, so weigh that knowingly.
    Blob nearest = candidates.front();
    for (auto const &blob : candidates)
    {
        if (distance(boat, blob.centre) < distance(boat, nearest.centre))
        {
            nearest = blob;
        }
    }

    locked_ = sane(nearest, settings_.max_object_radius_, node_);
    locked_at_ = node_->now();
    why_.clear();
    RCLCPP_INFO(node_->get_logger(), "locked on in front at (%.1f, %.1f), radius %.2f m", locked_->centre.x,
                locked_->centre.y, locked_->radius);
    return true;
}

bool TargetLock::refresh()
{
    if (!locked_)
    {
        why_ = "nothing locked on to refresh";
        return false;
    }

    // Tracking, not finding: use the tight gate (see refresh_max_jump in config/maneuvers.yaml).
    Match const match =
        match_nearest(real_only(blobs()), locked_->centre, settings_.refresh_max_jump_, settings_.ambiguous_margin_);

    if (!match.ok)
    {
        why_ = describe(match.failure);
        // Throttled: failures are normal here (blind wedges, dropped clustering frames) and a 3 Hz log buries
        // everything else. A lost lock is reported by the caller's stale() check.
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "refresh failed: %s (keeping the remembered point)", why_.c_str());
        return false;
    }

    double const moved = distance(match.blob.centre, locked_->centre);
    locked_ = sane(match.blob, settings_.max_object_radius_, node_);
    locked_at_ = node_->now();
    why_.clear();
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "refreshed, moved %.2f m to (%.1f, %.1f)",
                         moved, locked_->centre.x, locked_->centre.y);
    return true;
}

bool TargetLock::stale() const
{
    if (!locked_)
    {
        return true;
    }
    return (node_->now() - locked_at_).seconds() > settings_.reading_max_age_;
}

}  // namespace prop_maneuvers
