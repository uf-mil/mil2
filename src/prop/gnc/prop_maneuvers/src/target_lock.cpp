#include "prop_maneuvers/target_lock.hpp"

#include <cmath>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace prop_maneuvers
{
namespace
{
/// Keep an implausible cluster from sizing a standoff or a clearance.
///
/// The clustering merges returns now and then while the boat is moving, and a
/// merged blob's half-width is not the object's radius. Everything downstream
/// adds this to a distance, so one bad frame quietly moves the goalposts.
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
/// The blobs that could plausibly BE the object we are tracking.
///
/// Merged clusters are dropped here and nowhere else: they still have to be
/// avoided, so plan_detour and clear_behind keep seeing them, but adopting one
/// as the target hands the approach a phantom. Measured 2026-09-08: a merged
/// blob whose centroid sat within a metre of the real buoy captured the lock,
/// inflated the standoff to 4.50 m by way of its clamped radius, and stopped
/// the boat 1.24 m short while reporting that it had arrived.
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
            // Transform at the MARKER'S OWN STAMP, not the latest available.
            //
            // An earlier version forced rclcpp::Time(0) here on the reasoning
            // that the position chain updates faster than the lidar. That is
            // true and it is exactly the problem: pairing old lidar data with
            // a new pose swings every static object by roughly range times the
            // yaw that happened in between. Measured in simulation on
            // 2026-09-07, a stationary buoy 5.2 m away appeared to jump 1.93 m
            // in a single refresh while the boat turned -- about 22 degrees of
            // yaw, well under a second at the 0.6 rad/s cap. The lock followed
            // the phantom, the standoff point collapsed onto the boat, and the
            // approach reported "arrived" without moving. This is very likely
            // the same unexplained 1.4 m lock drift seen on an earlier run.
            //
            // tf2 interpolates within its buffer, so asking for the scan's own
            // time is both cheap and correct.
            out_point = tf_buffer_->transform(in, "map");
        }
        catch (tf2::TransformException const &error)
        {
            // Fall back to the latest transform rather than going blind. This
            // reintroduces the swing above, so it is a degraded mode and says
            // so: better a wobbly position than none while tf catches up.
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
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                     "cannot place blobs on the map yet: %s", fallback_error.what());
                return {};
            }
        }

        // The clustering publishes a box; treat the larger of its two ground
        // dimensions as a diameter.
        //
        // Clamped here, for EVERY consumer, not just the locked object. The
        // clustering merges returns over open water now and then and reports
        // one enormous blob: radii of 3.2 m and 5.2 m were seen on
        // 2026-09-07, and 9.3 m earlier the same evening. Left raw, such a
        // phantom does real damage -- it inflated an approach's standoff past
        // its own distance to the target, and it reached into the reverse
        // strip from 1.8 m off to the side and blocked backing off entirely,
        // leaving the boat stuck with nowhere it was willing to go.
        //
        // Clamping rather than discarding: something is there, or at least
        // might be, so it should still be avoided. It just must not be
        // believed about its size. Anything on this course is a buoy about
        // 0.46 m across, so max_object_radius is generous already.
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
    auto const all = blobs();
    // The hint must land within match_radius of the real buoy. Using the much
    // larger acquire_max_range here would make the ambiguity check fire almost
    // every time, because several buoys would sit inside it.
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

    // Nearest one in the cone wins. Ambiguity is not checked here: the caller
    // asked for whatever is in front, so picking the closest is the answer.
    // There is no prediction to disambiguate against, unlike acquire_near and
    // refresh, which lean on match_nearest's margin check. The tradeoff is
    // real: two buoys at similar range inside the forward cone resolve
    // silently to the nearer one, with no warning that the pick was close.
    // This is the one acquisition path meant for autonomous use on the real
    // boat, and the only one with no such protection -- a future reader
    // should weigh that knowingly rather than assume the ambiguity check
    // above applies here too.
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

    // Tracking, not finding: use the tight gate. See refresh_max_jump in
    // config/maneuvers.yaml for why this is not match_radius.
    Match const match =
        match_nearest(real_only(blobs()), locked_->centre, settings_.refresh_max_jump_, settings_.ambiguous_margin_);

    if (!match.ok)
    {
        why_ = describe(match.failure);
        RCLCPP_WARN(node_->get_logger(), "refresh failed: %s (keeping the remembered point)", why_.c_str());
        return false;
    }

    double const moved = distance(match.blob.centre, locked_->centre);
    locked_ = sane(match.blob, settings_.max_object_radius_, node_);
    locked_at_ = node_->now();
    why_.clear();
    RCLCPP_INFO(node_->get_logger(), "refreshed, moved %.2f m to (%.1f, %.1f)", moved, locked_->centre.x,
                locked_->centre.y);
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
