/**
 * @file pcl_clustering.hpp
 * @brief PclClustering — Euclidean cluster extraction on a pre-filtered cloud.
 *
 * Inherits all tunable parameters from PcdConstants (config/pcd_params.yaml).
 *
 * Expected input: the "filtered_cloud" topic published by PclFilter.
 *
 * Clustering pipeline:
 *   1. Build a kd-tree search structure.
 *   2. Run PCL EuclideanClusterExtraction with the configured
 *      tolerance / min / max thresholds.
 *   3. Publish a coloured PointCloud2 (one colour per cluster) and a
 *      MarkerArray of bounding boxes for RViz visualisation.
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "pcd/pcd_constants.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace pcd
{

// ── Internal colour palette (anonymous helper) ────────────────────────────
namespace detail
{

struct Rgb
{
    uint8_t r, g, b;
};

inline Rgb cluster_color(std::size_t index)
{
    static constexpr Rgb kPalette[] = {
        { 230, 25, 75 },  { 60, 180, 75 },  { 255, 225, 25 }, { 0, 130, 200 },   { 245, 130, 48 }, { 145, 30, 180 },
        { 70, 240, 240 }, { 240, 50, 230 }, { 210, 245, 60 }, { 250, 190, 212 }, { 0, 128, 128 },  { 220, 190, 255 },
    };
    return kPalette[index % (sizeof(kPalette) / sizeof(kPalette[0]))];
}

}  // namespace detail

/**
 * @class PclClustering
 * @brief ROS 2 node that subscribes to a pre-filtered PointCloud2 and
 *        performs Euclidean cluster extraction.
 *
 * Publishes:
 *   - "clusters_cloud"   (sensor_msgs/PointCloud2)  — coloured per-cluster cloud
 *   - "cluster_markers"  (visualization_msgs/MarkerArray) — bounding-box markers
 *
 * Inherits constants from PcdConstants.
 */
class PclClustering : public rclcpp::Node, public PcdConstants
{
  public:
    PclClustering() : rclcpp::Node("pcl_clustering"), PcdConstants(this)
    {
        // Listen on the filtered cloud produced by PclFilter.
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "filtered_cloud", rclcpp::SensorDataQoS(),
            std::bind(&PclClustering::cloud_cb, this, std::placeholders::_1));

        pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("clusters_cloud", rclcpp::QoS(1));
        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("cluster_markers", rclcpp::QoS(1));

        RCLCPP_INFO(get_logger(), "pcl_clustering started — tolerance=%.2f m  min=%d  max=%d pts", cluster_tolerance_,
                    cluster_min_points_, cluster_max_points_);
    }

  private:
    // ── Callback ──────────────────────────────────────────────────────────────

    void cloud_cb(sensor_msgs::msg::PointCloud2::ConstSharedPtr const msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Received empty cloud — no clustering performed");
            publish_empty_markers(msg->header);
            return;
        }

        // Build kd-tree.
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        // Extract clusters.
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(static_cast<float>(cluster_tolerance_));
        ec.setMinClusterSize(cluster_min_points_);
        ec.setMaxClusterSize(cluster_max_points_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Found %zu cluster(s) from %zu point(s)",
                             cluster_indices.size(), cloud->size());

        publish_clusters(msg->header, cloud, cluster_indices);
    }

    // ── Publishing helpers ────────────────────────────────────────────────────

    void publish_empty_markers(std_msgs::msg::Header const &header)
    {
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker clear;
        clear.header = header;
        clear.ns = "pcd_clusters";
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clear);
        pub_markers_->publish(markers);
    }

    void publish_clusters(std_msgs::msg::Header const &header, pcl::PointCloud<pcl::PointXYZ>::ConstPtr const &cloud,
                          std::vector<pcl::PointIndices> const &cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZRGB> colored;
        colored.reserve(cloud->size());

        visualization_msgs::msg::MarkerArray markers;
        // First marker clears stale boxes from the previous frame.
        visualization_msgs::msg::Marker clear;
        clear.header = header;
        clear.ns = "pcd_clusters";
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clear);

        int id = 0;
        for (auto const &cluster : cluster_indices)
        {
            detail::Rgb const color = detail::cluster_color(static_cast<std::size_t>(id));

            // Accumulate bounding-box extents and centroid.
            float min_x = std::numeric_limits<float>::max();
            float min_y = std::numeric_limits<float>::max();
            float min_z = std::numeric_limits<float>::max();
            float max_x = std::numeric_limits<float>::lowest();
            float max_y = std::numeric_limits<float>::lowest();
            float max_z = std::numeric_limits<float>::lowest();
            double cx = 0.0, cy = 0.0, cz = 0.0;

            for (int const idx : cluster.indices)
            {
                auto const &pt = (*cloud)[static_cast<std::size_t>(idx)];

                pcl::PointXYZRGB rgb_pt;
                rgb_pt.x = pt.x;
                rgb_pt.y = pt.y;
                rgb_pt.z = pt.z;
                rgb_pt.r = color.r;
                rgb_pt.g = color.g;
                rgb_pt.b = color.b;
                colored.push_back(rgb_pt);

                min_x = std::min(min_x, pt.x);
                max_x = std::max(max_x, pt.x);
                min_y = std::min(min_y, pt.y);
                max_y = std::max(max_y, pt.y);
                min_z = std::min(min_z, pt.z);
                max_z = std::max(max_z, pt.z);
                cx += pt.x;
                cy += pt.y;
                cz += pt.z;
            }

            double const n = static_cast<double>(cluster.indices.size());
            cx /= n;
            cy /= n;
            cz /= n;

            // Bounding-box marker.
            visualization_msgs::msg::Marker box;
            box.header = header;
            box.ns = "pcd_clusters";
            box.id = id;
            box.type = visualization_msgs::msg::Marker::CUBE;
            box.action = visualization_msgs::msg::Marker::ADD;
            box.pose.position.x = cx;
            box.pose.position.y = cy;
            box.pose.position.z = cz;
            box.pose.orientation.w = 1.0;
            box.scale.x = std::max(0.05, static_cast<double>(max_x - min_x));
            box.scale.y = std::max(0.05, static_cast<double>(max_y - min_y));
            box.scale.z = std::max(0.05, static_cast<double>(max_z - min_z));
            box.color.r = color.r / 255.0f;
            box.color.g = color.g / 255.0f;
            box.color.b = color.b / 255.0f;
            box.color.a = 0.35f;
            box.lifetime = rclcpp::Duration(0, 0);
            markers.markers.push_back(box);
            ++id;
        }

        colored.width = static_cast<uint32_t>(colored.size());
        colored.height = 1;
        colored.is_dense = true;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(colored, cloud_msg);
        cloud_msg.header = header;

        pub_cloud_->publish(cloud_msg);
        pub_markers_->publish(markers);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
};

}  // namespace pcd
