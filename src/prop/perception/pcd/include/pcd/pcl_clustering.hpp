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
#include <cmath>
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

/// The footprint of a cluster as an oriented rectangle.
struct OrientedBox
{
    double x{ 0.0 }, y{ 0.0 }, z{ 0.0 };  ///< centre of the box, not the centroid
    double yaw{ 0.0 };                    ///< heading of the long axis
    double length{ 0.0 };                 ///< extent along yaw
    double width{ 0.0 };                  ///< extent across it
    double height{ 0.0 };
};

/// Fit an oriented box to a cluster's footprint.
///
/// The axis comes from the 2-D principal component of the member points, which
/// is what makes the box mean something. An axis-aligned box carries no
/// heading of its own, so once its centre is transformed into a world frame
/// its extents still describe whichever way the boat happened to be pointing
/// when the sweep was taken - and a dock would read as a different shape on
/// every heading. The principal axis is a property of the object.
inline OrientedBox fit_oriented_box(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const &cloud,
                                    pcl::PointIndices const &cluster)
{
    double const n = static_cast<double>(cluster.indices.size());

    double mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (int const idx : cluster.indices)
    {
        auto const &pt = (*cloud)[static_cast<std::size_t>(idx)];
        mean_x += pt.x;
        mean_y += pt.y;
        mean_z += pt.z;
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
    }
    mean_x /= n;
    mean_y /= n;
    mean_z /= n;

    // Second moments about the mean; the principal axis is the eigenvector of
    // the larger eigenvalue, which for a symmetric 2x2 reduces to one atan2.
    double cxx = 0.0, cyy = 0.0, cxy = 0.0;
    for (int const idx : cluster.indices)
    {
        auto const &pt = (*cloud)[static_cast<std::size_t>(idx)];
        double const dx = pt.x - mean_x;
        double const dy = pt.y - mean_y;
        cxx += dx * dx;
        cyy += dy * dy;
        cxy += dx * dy;
    }
    double const yaw = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
    double const ux = std::cos(yaw);
    double const uy = std::sin(yaw);

    // Extents along the axis and across it.
    double min_a = std::numeric_limits<double>::max();
    double max_a = std::numeric_limits<double>::lowest();
    double min_b = std::numeric_limits<double>::max();
    double max_b = std::numeric_limits<double>::lowest();
    for (int const idx : cluster.indices)
    {
        auto const &pt = (*cloud)[static_cast<std::size_t>(idx)];
        double const dx = pt.x - mean_x;
        double const dy = pt.y - mean_y;
        double const a = dx * ux + dy * uy;
        double const b = -dx * uy + dy * ux;
        min_a = std::min(min_a, a);
        max_a = std::max(max_a, a);
        min_b = std::min(min_b, b);
        max_b = std::max(max_b, b);
    }

    // The centre of the extents, which is not the centroid: a partly seen
    // object has its returns bunched on the face the lidar can reach.
    double const mid_a = 0.5 * (min_a + max_a);
    double const mid_b = 0.5 * (min_b + max_b);

    OrientedBox box;
    box.x = mean_x + mid_a * ux - mid_b * uy;
    box.y = mean_y + mid_a * uy + mid_b * ux;
    box.z = mean_z;
    box.yaw = yaw;
    box.length = max_a - min_a;
    box.width = max_b - min_b;
    box.height = static_cast<double>(max_z - min_z);
    return box;
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
            }

            detail::OrientedBox const box = detail::fit_oriented_box(cloud, cluster);

            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "pcd_clusters";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = box.x;
            marker.pose.position.y = box.y;
            marker.pose.position.z = box.z;
            marker.pose.orientation.z = std::sin(0.5 * box.yaw);
            marker.pose.orientation.w = std::cos(0.5 * box.yaw);
            marker.scale.x = std::max(0.05, box.length);
            marker.scale.y = std::max(0.05, box.width);
            marker.scale.z = std::max(0.05, box.height);
            marker.color.r = color.r / 255.0f;
            marker.color.g = color.g / 255.0f;
            marker.color.b = color.b / 255.0f;
            marker.color.a = 0.35f;
            marker.lifetime = rclcpp::Duration(0, 0);
            markers.markers.push_back(marker);
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

        RCLCPP_INFO(get_logger(), "Published %zu cluster(s) with %zu colored point(s) and %zu marker(s)",
                    cluster_indices.size(), colored.size(), markers.markers.size());
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
};

}  // namespace pcd
