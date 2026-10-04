/**
 * @file pcl_filter.hpp
 * @brief PclFilter — pass-through distance filter and water-surface rejection.
 *
 * Inherits all tunable parameters from PcdConstants (config/pcd_params.yaml).
 *
 * Filter pipeline (applied in order):
 *   1. Radial distance pass-through  — rejects points outside [min_distance, max_distance].
 *   2. Z-height pass-through         — rejects water returns below water_z_min and
 *                                      sky noise above water_z_max, measured LEVEL
 *                                      rather than in the raw, possibly-tilted sensor
 *                                      frame (see apply_z_filter).
 *   3. Voxel-grid down-sample        — reduces density before clustering (optional).
 */

#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>

#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "pcd/pcd_constants.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace pcd
{

/// Roll and pitch only — yaw does not change how high a point sits above the
/// water, and translation does not enter a height cut at all.
struct Attitude
{
    double roll;
    double pitch;
};

/**
 * @class PclFilter
 * @brief ROS 2 node that subscribes to a raw PointCloud2 topic, applies
 *        distance and height filtering, optionally voxel-downsamples, and
 *        re-publishes the cleaned cloud for downstream clustering.
 *
 * Inherits constants from PcdConstants.
 */
class PclFilter : public rclcpp::Node, public PcdConstants
{
  public:
    PclFilter() : rclcpp::Node("pcl_filter"), PcdConstants(this)
    {
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS(), std::bind(&PclFilter::cloud_cb, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(), std::bind(&PclFilter::imu_cb, this, std::placeholders::_1));

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", rclcpp::QoS(1));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(get_logger(),
                    "pcl_filter started — topic='%s'  imu='%s'  dist=[%.2f, %.2f] m  "
                    "z=[%.2f, %.2f] m (levelled by roll/pitch)  voxel=%.3f m",
                    input_topic_.c_str(), imu_topic_.c_str(), min_distance_, max_distance_, water_z_min_, water_z_max_,
                    voxel_leaf_size_);
    }

  private:
    /// How long a window of IMU samples to keep buffered, independent of
    /// attitude_max_age_: a generous, fixed bound so tuning that parameter up
    /// cannot silently grow the buffer without limit.
    static constexpr double kImuBufferWindowSec = 2.0;

    // ── Callbacks ────────────────────────────────────────────────────────────

    void imu_cb(sensor_msgs::msg::Imu::ConstSharedPtr const msg)
    {
        imu_buffer_.push_back(*msg);
        rclcpp::Time const newest(imu_buffer_.back().header.stamp);
        while (!imu_buffer_.empty() &&
               (newest - rclcpp::Time(imu_buffer_.front().header.stamp)).seconds() > kImuBufferWindowSec)
        {
            imu_buffer_.pop_front();
        }
    }

    void cloud_cb(sensor_msgs::msg::PointCloud2::ConstSharedPtr const msg)
    {
        // Convert to PCL.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Received empty cloud — skipping");
            return;
        }

        auto filtered = apply_distance_filter(cloud);
        filtered = apply_z_filter(filtered, attitude_at(msg->header.stamp, msg->header.frame_id));
        filtered = apply_voxel(filtered);

        if (filtered->empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Cloud is empty after filtering — nothing to publish");
            return;
        }

        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*filtered, out_msg);
        out_msg.header = msg->header;
        pub_->publish(out_msg);
    }

    // ── Filter helpers ────────────────────────────────────────────────────────

    /**
     * @brief Radial distance filter.
     *
     * Computes the Euclidean distance of each point from the sensor origin and
     * discards those outside [min_distance_, max_distance_].
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_distance_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr const &in) const
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
        out->reserve(in->size());
        for (auto const &pt : *in)
        {
            double const r = std::sqrt(static_cast<double>(pt.x) * pt.x + static_cast<double>(pt.y) * pt.y +
                                       static_cast<double>(pt.z) * pt.z);
            if (r >= min_distance_ && r <= max_distance_)
            {
                out->push_back(pt);
            }
        }
        out->width = static_cast<uint32_t>(out->size());
        out->height = 1;
        out->is_dense = true;
        return out;
    }

    /**
     * @brief Z-height pass-through filter (rejects water returns and sky noise).
     *
     * Keeps points whose LEVELLED z lies in [water_z_min_, water_z_max_].
     *
     * A constant z cut taken directly in the sensor frame is only valid while
     * the sensor is level. Circling pitches the boat 0.3-0.5 degrees, which
     * tilts the water plane inside the lidar frame enough that a water point
     * at 25 m rises 0.17 m -- clean over a -0.50 m cut. Measured 2026-09-13,
     * replaying 12 captured mid-circle frames offline: that let 500-3400 water
     * points a frame survive, which Euclidean clustering then shattered into
     * 18-22 clusters, some over 11 m across. Levelling first (this function)
     * collapsed that to a median of 2 clusters, matching a level boat.
     *
     * Rotating by roll then pitch and reading off the resulting z is
     * equivalent to rotating the whole cloud and keeping only the z it
     * produces -- yaw does not change a point's height and translation does
     * not enter a height cut at all, so this needs nothing else. The point
     * itself is returned UNCHANGED; only the test against water_z_min_ /
     * water_z_max_ is levelled, so everything downstream keeps seeing the
     * sensor frame the header claims.
     *
     * When `attitude` is absent -- no IMU sample young enough to trust, see
     * attitude_at() -- this degrades to the old, unlevelled behaviour rather
     * than guessing.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_z_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr const &in,
                                                       std::optional<Attitude> const &attitude) const
    {
        double const roll = attitude ? attitude->roll : 0.0;
        double const pitch = attitude ? attitude->pitch : 0.0;
        double const cr = std::cos(roll);
        double const sr = std::sin(roll);
        double const cp = std::cos(pitch);
        double const sp = std::sin(pitch);

        pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
        out->reserve(in->size());
        for (auto const &pt : *in)
        {
            double const z_level = -sp * pt.x + cp * sr * pt.y + cp * cr * pt.z;
            if (z_level >= water_z_min_ && z_level <= water_z_max_)
            {
                out->push_back(pt);
            }
        }
        out->width = static_cast<uint32_t>(out->size());
        out->height = 1;
        out->is_dense = true;
        return out;
    }

    /**
     * @brief Roll and pitch OF THE CLOUD'S FRAME, from the IMU sample closest
     *        in time to `stamp`, within attitude_max_age_.
     *
     * The IMU reports its own orientation, not the lidar's, and the two are not
     * mounted alike: prop.urdf yaws the IMU by pi ("mounted backwards"), which
     * flips the sign of both its roll and its pitch relative to the boat.
     * Levelling by the raw IMU angles therefore tilted the cloud the wrong way.
     * Measured 2026-09-28 on 160 captured turning frames (4.5 deg nose-up):
     * raw-IMU levelling let a median 7334 water points through a -0.35 cut;
     * the same frames levelled in the cloud's frame let through 0. So the IMU
     * orientation is carried into the cloud frame through TF first.
     *
     * Matched by nearest timestamp rather than "whatever arrived last": an
     * offline replay of this exact frame set, keyed on the last-received
     * attitude instead of the one at the cloud's own stamp, reproduced the
     * same stale-sample failure this function exists to avoid -- one frame in
     * twelve still shattered (14 clusters instead of 2) because the attitude
     * it was paired with was not the one the boat actually had when the cloud
     * was captured.
     *
     * Returns std::nullopt if the buffer is empty, every sample is older or
     * newer than attitude_max_age_ from `stamp`, or TF has no transform between
     * the IMU and cloud frames, so callers fail safe to the unlevelled z cut
     * instead of levelling by a stale, absent or wrongly-mounted attitude.
     */
    std::optional<Attitude> attitude_at(builtin_interfaces::msg::Time const &stamp,
                                        std::string const &cloud_frame) const
    {
        if (imu_buffer_.empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "no IMU samples on '%s' yet; using the unlevelled z cut", imu_topic_.c_str());
            return std::nullopt;
        }

        rclcpp::Time const target(stamp);
        sensor_msgs::msg::Imu const *best = nullptr;
        double best_age = std::numeric_limits<double>::infinity();
        for (auto const &sample : imu_buffer_)
        {
            double const age = std::abs((rclcpp::Time(sample.header.stamp) - target).seconds());
            if (age < best_age)
            {
                best_age = age;
                best = &sample;
            }
        }

        if (best == nullptr || best_age > attitude_max_age_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "nearest IMU sample is %.3f s from the cloud (limit %.3f s); using the "
                                 "unlevelled z cut",
                                 best_age, attitude_max_age_);
            return std::nullopt;
        }

        // Mounting only, so the latest (static) transform is the right one.
        geometry_msgs::msg::TransformStamped imu_from_cloud;
        try
        {
            imu_from_cloud = tf_buffer_->lookupTransform(best->header.frame_id, cloud_frame, tf2::TimePointZero);
        }
        catch (tf2::TransformException const &e)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "no transform from cloud frame '%s' to IMU frame '%s' (%s); using the "
                                 "unlevelled z cut",
                                 cloud_frame.c_str(), best->header.frame_id.c_str(), e.what());
            return std::nullopt;
        }

        // world <- cloud = (world <- imu) * (imu <- cloud)
        tf2::Quaternion const world_from_imu(best->orientation.x, best->orientation.y, best->orientation.z,
                                             best->orientation.w);
        auto const &r = imu_from_cloud.transform.rotation;
        tf2::Quaternion const imu_from_cloud_q(r.x, r.y, r.z, r.w);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(world_from_imu * imu_from_cloud_q).getRPY(roll, pitch, yaw);
        return Attitude{ roll, pitch };
    }

    /**
     * @brief Optional voxel-grid down-sampling.
     *
     * Skipped when voxel_leaf_size_ <= 0.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr const &in) const
    {
        if (voxel_leaf_size_ <= 0.0)
        {
            return in;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(in);
        float const leaf = static_cast<float>(voxel_leaf_size_);
        vg.setLeafSize(leaf, leaf, leaf);
        vg.filter(*out);
        return out;
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    /// Recent IMU samples, oldest first, trimmed to kImuBufferWindowSec.
    std::deque<sensor_msgs::msg::Imu> imu_buffer_;
    /// For the IMU-to-cloud mounting rotation.
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace pcd
