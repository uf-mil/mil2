#include "prop_maneuvers/target_lock.hpp"

#include <cmath>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace prop_maneuvers
{

TargetLock::TargetLock(rclcpp::Node *node, Constants const &settings)
  : node_(node), settings_(settings), locked_at_(node->now())
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    markers_subscription_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        "cluster_markers", rclcpp::QoS(1),
        [this](visualization_msgs::msg::MarkerArray::SharedPtr const msg) { latest_ = *msg; });
}

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
            // Use the latest available transform rather than the marker's own
            // stamp: the position chain updates faster than the lidar, and an
            // exact-time lookup fails more often than it helps here.
            in.header.stamp = rclcpp::Time(0);
            out_point = tf_buffer_->transform(in, "map");
        }
        catch (tf2::TransformException const &error)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "cannot place blobs on the map yet: %s", error.what());
            return {};
        }

        // The clustering publishes a box; treat the larger of its two ground
        // dimensions as a diameter.
        double const width = std::max(marker.scale.x, marker.scale.y);
        out.push_back(Blob{ Point{ out_point.point.x, out_point.point.y }, width / 2.0 });
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

    locked_ = match.blob;
    locked_at_ = node_->now();
    why_.clear();
    RCLCPP_INFO(node_->get_logger(), "locked on at (%.1f, %.1f), radius %.2f m", locked_->centre.x, locked_->centre.y,
                locked_->radius);
    return true;
}

bool TargetLock::acquire_in_front(Point const &boat, double boat_direction)
{
    std::vector<Blob> candidates;
    for (auto const &blob : blobs())
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
    Blob nearest = candidates.front();
    for (auto const &blob : candidates)
    {
        if (distance(boat, blob.centre) < distance(boat, nearest.centre))
        {
            nearest = blob;
        }
    }

    locked_ = nearest;
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

    Match const match = match_nearest(blobs(), locked_->centre, settings_.match_radius_, settings_.ambiguous_margin_);

    if (!match.ok)
    {
        why_ = describe(match.failure);
        RCLCPP_WARN(node_->get_logger(), "refresh failed: %s (keeping the remembered point)", why_.c_str());
        return false;
    }

    double const moved = distance(match.blob.centre, locked_->centre);
    locked_ = match.blob;
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
