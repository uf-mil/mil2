/**
 * Bare-bones Euclidean cluster extraction for lidar PointCloud2 streams.
 *
 * Follows the Point Cloud Library tutorial:
 * https://pointclouds.org/documentation/tutorials/cluster_extraction.html
 *
 * Pipeline: subscribe -> (optional) voxel downsample -> (optional) plane
 * removal -> EuclideanClusterExtraction -> MarkerArray + colored cloud.
 */

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{

struct Rgb
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

Rgb cluster_color(std::size_t index)
{
    // Distinct hues cycling through HSV-ish primaries for RViz.
    static Rgb const kPalette[] = {
        { 230, 25, 75 },  { 60, 180, 75 },  { 255, 225, 25 }, { 0, 130, 200 },   { 245, 130, 48 }, { 145, 30, 180 },
        { 70, 240, 240 }, { 240, 50, 230 }, { 210, 245, 60 }, { 250, 190, 212 }, { 0, 128, 128 },  { 220, 190, 255 },
    };
    return kPalette[index % (sizeof(kPalette) / sizeof(kPalette[0]))];
}

}  // namespace

class EuclideanClusterNode : public rclcpp::Node
{
  public:
    EuclideanClusterNode() : Node("euclidean_cluster_node")
    {
        declare_parameter<std::string>("input_topic", "/velodyne_points");
        declare_parameter("voxel_leaf_size", 0.1);
        declare_parameter("remove_planes", true);
        declare_parameter("plane_distance_threshold", 0.2);
        declare_parameter("plane_max_iterations", 100);
        declare_parameter("min_plane_fraction", 0.3);
        declare_parameter("cluster_tolerance", 0.5);
        declare_parameter("cluster_min_points", 20);
        declare_parameter("cluster_max_points", 25000);

        input_topic_ = get_parameter("input_topic").as_string();
        voxel_leaf_size_ = get_parameter("voxel_leaf_size").as_double();
        remove_planes_ = get_parameter("remove_planes").as_bool();
        plane_distance_threshold_ = get_parameter("plane_distance_threshold").as_double();
        plane_max_iterations_ = get_parameter("plane_max_iterations").as_int();
        min_plane_fraction_ = get_parameter("min_plane_fraction").as_double();
        cluster_tolerance_ = get_parameter("cluster_tolerance").as_double();
        cluster_min_points_ = get_parameter("cluster_min_points").as_int();
        cluster_max_points_ = get_parameter("cluster_max_points").as_int();

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&EuclideanClusterNode::cloud_cb, this, std::placeholders::_1));

        pub_clusters_ = create_publisher<sensor_msgs::msg::PointCloud2>("clusters_cloud", rclcpp::QoS(1));
        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("cluster_markers", rclcpp::QoS(1));

        RCLCPP_INFO(get_logger(),
                    "euclidean_cluster_node listening on %s "
                    "(voxel=%.3f, remove_planes=%s, tolerance=%.3f, min=%d, max=%d)",
                    input_topic_.c_str(), voxel_leaf_size_, remove_planes_ ? "true" : "false", cluster_tolerance_,
                    cluster_min_points_, cluster_max_points_);
    }

  private:
    void cloud_cb(sensor_msgs::msg::PointCloud2::ConstSharedPtr const msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Empty input cloud");
            return;
        }

        // Voxel downsample (tutorial leaf size is 1cm; lidar sim uses a larger default).
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        if (voxel_leaf_size_ > 0.0)
        {
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(cloud);
            float const leaf = static_cast<float>(voxel_leaf_size_);
            vg.setLeafSize(leaf, leaf, leaf);
            vg.filter(*cloud_filtered);
        }
        else
        {
            *cloud_filtered = *cloud;
        }

        if (cloud_filtered->empty())
        {
            return;
        }

        // Optional repeated plane removal (same loop structure as the tutorial).
        if (remove_planes_)
        {
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(plane_max_iterations_);
            seg.setDistanceThreshold(plane_distance_threshold_);

            int const nr_points = static_cast<int>(cloud_filtered->size());
            while (cloud_filtered->size() > min_plane_fraction_ * nr_points)
            {
                seg.setInputCloud(cloud_filtered);
                seg.segment(*inliers, *coefficients);
                if (inliers->indices.empty())
                {
                    break;
                }

                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud_filtered);
                extract.setIndices(inliers);
                extract.setNegative(true);
                extract.filter(*cloud_f);
                *cloud_filtered = *cloud_f;
            }
        }

        if (cloud_filtered->empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No points left after filtering / plane removal");
            publish_empty_markers(msg->header);
            return;
        }

        // Euclidean cluster extraction (tutorial core).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(cluster_min_points_);
        ec.setMaxClusterSize(cluster_max_points_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Found %zu clusters (%zu points after filter)",
                             cluster_indices.size(), cloud_filtered->size());

        publish_clusters(msg->header, cloud_filtered, cluster_indices);
    }

    void publish_empty_markers(std_msgs::msg::Header const &header)
    {
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker clear;
        clear.header = header;
        clear.ns = "euclidean_clusters";
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
        visualization_msgs::msg::Marker clear;
        clear.header = header;
        clear.ns = "euclidean_clusters";
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clear);

        int id = 0;
        for (auto const &cluster : cluster_indices)
        {
            Rgb const color = cluster_color(static_cast<std::size_t>(id));

            float min_x = std::numeric_limits<float>::max();
            float min_y = std::numeric_limits<float>::max();
            float min_z = std::numeric_limits<float>::max();
            float max_x = std::numeric_limits<float>::lowest();
            float max_y = std::numeric_limits<float>::lowest();
            float max_z = std::numeric_limits<float>::lowest();
            double cx = 0.0;
            double cy = 0.0;
            double cz = 0.0;

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
                min_y = std::min(min_y, pt.y);
                min_z = std::min(min_z, pt.z);
                max_x = std::max(max_x, pt.x);
                max_y = std::max(max_y, pt.y);
                max_z = std::max(max_z, pt.z);
                cx += pt.x;
                cy += pt.y;
                cz += pt.z;
            }

            double const n = static_cast<double>(cluster.indices.size());
            cx /= n;
            cy /= n;
            cz /= n;

            visualization_msgs::msg::Marker box;
            box.header = header;
            box.ns = "euclidean_clusters";
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

        colored.width = colored.size();
        colored.height = 1;
        colored.is_dense = true;

        sensor_msgs::msg::PointCloud2 colored_msg;
        pcl::toROSMsg(colored, colored_msg);
        colored_msg.header = header;
        pub_clusters_->publish(colored_msg);
        pub_markers_->publish(markers);
    }

    std::string input_topic_;
    double voxel_leaf_size_{ 0.1 };
    bool remove_planes_{ true };
    double plane_distance_threshold_{ 0.2 };
    int plane_max_iterations_{ 100 };
    double min_plane_fraction_{ 0.3 };
    double cluster_tolerance_{ 0.5 };
    int cluster_min_points_{ 20 };
    int cluster_max_points_{ 25000 };

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clusters_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EuclideanClusterNode>());
    rclcpp::shutdown();
    return 0;
}
