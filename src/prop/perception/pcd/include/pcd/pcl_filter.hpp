/**
 * @file pcl_filter.hpp
 * @brief PclFilter — pass-through distance filter and water-surface rejection.
 *
 * Inherits all tunable parameters from PcdConstants (config/pcd_params.yaml).
 *
 * Filter pipeline (applied in order):
 *   1. Radial distance pass-through  — rejects points outside [min_distance, max_distance].
 *   2. Z-height pass-through         — rejects water returns below water_z_min and
 *                                      sky noise above water_z_max.
 *   3. Voxel-grid down-sample        — reduces density before clustering (optional).
 */

#pragma once

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "pcd/pcd_constants.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pcd
{

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

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", rclcpp::QoS(1));

        RCLCPP_INFO(get_logger(),
                    "pcl_filter started — topic='%s'  dist=[%.2f, %.2f] m  "
                    "z=[%.2f, %.2f] m  voxel=%.3f m",
                    input_topic_.c_str(), min_distance_, max_distance_, water_z_min_, water_z_max_, voxel_leaf_size_);
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
        filtered = apply_z_filter(filtered);
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
     * Keeps points with  water_z_min_ <= z <= water_z_max_.
     * All points below water_z_min_ are treated as water-surface reflections
     * and are discarded.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr apply_z_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr const &in) const
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(in);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(static_cast<float>(water_z_min_), static_cast<float>(water_z_max_));
        pass.filter(*out);
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
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace pcd
