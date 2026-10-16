/**
 * @file pcl_filter.hpp
 * @brief PclFilter — pass-through distance filter, water-surface rejection,
 *        and TF transform to target_frame (odom).
 *
 * Inherits all tunable parameters from PcdConstants (config/pcd_params.yaml)
 *
 * Filter pipeline (applied in order):
 *   1. Radial distance pass-through  — rejects points outside [min_distance, max_distance].
 *   2. Height above the water plane  — rejects water and sky; the plane is
 *                                      levelled with the IMU orientation.
 *   3. Voxel-grid down-sample        — reduces density before clustering (optional).
 *   4. TF transform                  — reprojects the filtered cloud from velodyne
 *                                      frame into target_frame (odom) so that
 *                                      downstream clustering and tracking work in
 *                                      a stable global frame. An empty
 *                                      target_frame keeps the sensor frame.
 */

#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "pcd/pcd_constants.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace pcd
{

/**
 * @class PclFilter
 * @brief ROS 2 node that subscribes to a raw PointCloud2 topic, applies
 *        distance and height filtering, optionally voxel-downsamples, then
 *        transforms the cloud to target_frame (odom) and re-publishes it.
 *
 * Inherits constants from PcdConstants.
 */
class PclFilter : public rclcpp::Node, public PcdConstants
{
  public:
    PclFilter() : rclcpp::Node("pcl_filter"), PcdConstants(this)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS(), std::bind(&PclFilter::cloud_cb, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS(),
                                                              [this](sensor_msgs::msg::Imu::ConstSharedPtr const msg)
                                                              { imu_ = msg; });

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", rclcpp::QoS(1));

        RCLCPP_INFO(get_logger(),
                    "pcl_filter started — topic='%s'  dist=[%.2f, %.2f] m  "
                    "height above water=[%.2f, %.2f] m  voxel=%.3f m  target_frame='%s'",
                    input_topic_.c_str(), min_distance_, max_distance_, min_height_above_water_,
                    max_height_above_water_, voxel_leaf_size_, target_frame_.c_str());
    }

  private:
    // ── Callback ─────────────────────────────────────────────────────────────

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
        filtered = apply_height_filter(filtered, msg->header);
        filtered = apply_voxel(filtered);

        if (filtered->empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Cloud is empty after filtering — nothing to publish");
            return;
        }

        // Convert filtered PCL cloud back to ROS message (still in velodyne frame).
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered, filtered_msg);
        filtered_msg.header = msg->header;

        if (target_frame_.empty())
        {
            pub_->publish(filtered_msg);
            return;
        }

        // Transform from velodyne frame into target_frame (odom).
        sensor_msgs::msg::PointCloud2 out_msg;
        try
        {
            out_msg = tf_buffer_->transform(filtered_msg, target_frame_, tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException const &ex)
        {
            try
            {
                // Fall back to latest available transform (zero timestamp).
                filtered_msg.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
                out_msg = tf_buffer_->transform(filtered_msg, target_frame_, tf2::durationFromSec(0.0));
            }
            catch (tf2::TransformException const &ex2)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "TF transform '%s'→'%s' failed: %s — dropping cloud", msg->header.frame_id.c_str(),
                                     target_frame_.c_str(), ex2.what());
                return;
            }
        }

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

    /// World up in the lidar frame, from the latest IMU orientation. Falls back
    /// to the lidar's own Z, which is only right while the hull is level.
    tf2::Vector3 up_in_lidar(std_msgs::msg::Header const &cloud) const
    {
        tf2::Vector3 const level{ 0.0, 0.0, 1.0 };
        if (!imu_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no IMU on %s yet, assuming the hull is level",
                                 imu_topic_.c_str());
            return level;
        }
        double const age = std::abs((rclcpp::Time(cloud.stamp) - rclcpp::Time(imu_->header.stamp)).seconds());
        if (age > max_imu_age_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "IMU orientation is %.2f s from the cloud, assuming the hull is level", age);
            return level;
        }
        tf2::Quaternion imu_in_world;
        tf2::fromMsg(imu_->orientation, imu_in_world);
        // An IMU without orientation sends zeros, which would zero every height and pass all the water.
        if (std::abs(imu_in_world.length() - 1.0) > 0.1)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "IMU orientation is not a rotation, assuming the hull is level");
            return level;
        }
        try
        {
            auto const mount = tf_buffer_->lookupTransform(cloud.frame_id, imu_->header.frame_id, tf2::TimePointZero);
            tf2::Quaternion imu_in_lidar;
            tf2::fromMsg(mount.transform.rotation, imu_in_lidar);
            tf2::Vector3 up_in_imu = tf2::quatRotate(imu_in_world.normalized().inverse(), level);
            double const facing = static_cast<double>(static_cast<int>(imu_facing_));
            up_in_imu.setX(facing * up_in_imu.x());
            up_in_imu.setY(facing * up_in_imu.y());
            return tf2::quatRotate(imu_in_lidar, up_in_imu);
        }
        catch (tf2::TransformException const &ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no %s -> %s, assuming the hull is level: %s",
                                 imu_->header.frame_id.c_str(), cloud.frame_id.c_str(), ex.what());
            return level;
        }
    }

    /// Keep returns between min and max height above the water plane.
    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_height_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr const &in,
                                                            std_msgs::msg::Header const &cloud) const
    {
        tf2::Vector3 const up = up_in_lidar(cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
        out->reserve(in->size());
        for (auto const &pt : *in)
        {
            double const height = pt.x * up.x() + pt.y * up.y() + pt.z * up.z() + lidar_height_;
            if (height >= min_height_above_water_ && height <= max_height_above_water_)
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
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    sensor_msgs::msg::Imu::ConstSharedPtr imu_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace pcd
