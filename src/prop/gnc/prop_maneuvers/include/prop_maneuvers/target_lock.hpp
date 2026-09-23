/**
 * @file target_lock.hpp
 * @brief Holds one remembered buoy position in map coordinates.
 *
 *   in  cluster_markers  visualization_msgs/MarkerArray, from pcd clustering
 *
 * No tracker and no persistent ID numbers are involved. The boat always knows
 * where it is, so one sighting converted into map coordinates is enough: the
 * position estimate carries the remembered point while the buoy is out of
 * sight, and a refresh matches the nearest blob to the predicted position.
 *
 * NOTE: cluster_markers is a display message being used as a data interface.
 * It carries the centre, the size and the frame, which is all that is needed,
 * and using it costs no coordination with the clustering's owner. If a proper
 * message appears later, only this file changes.
 */

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/constants.hpp"
#include "prop_maneuvers/geometry.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

namespace prop_maneuvers
{

class TargetLock
{
  public:
    TargetLock(rclcpp::Node *node, Constants const &settings);

    /// Every blob from the most recent clustering, in map coordinates.
    /// Empty when nothing has arrived yet or the transform is unavailable.
    std::vector<Blob> blobs() const;

    /// Lock onto the blob nearest `hint`, a rough map position from the caller.
    bool acquire_near(Point const &hint);

    /// Lock onto the nearest blob inside the forward cone, within range.
    bool acquire_in_front(Point const &boat, double boat_direction);

    /// Re-match against the remembered point and update it.
    bool refresh();

    bool locked() const
    {
        return locked_.has_value();
    }

    /// True when the lock has not been refreshed recently enough.
    bool stale() const;

    /// The remembered point. Only meaningful when locked().
    Point point() const
    {
        return locked_ ? locked_->centre : Point{};
    }

    /// The remembered blob's radius. Only meaningful when locked().
    double radius() const
    {
        return locked_ ? locked_->radius : 0.0;
    }

    /// Why the last acquire or refresh failed, for logging.
    std::string const &why() const
    {
        return why_;
    }

  private:
    rclcpp::Node *node_;
    Constants const &settings_;

    std::optional<Blob> locked_;
    rclcpp::Time locked_at_;
    std::string why_;

    visualization_msgs::msg::MarkerArray latest_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markers_subscription_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace prop_maneuvers
