/**
 * @file pcd_constants.hpp
 * @brief Base class that declares and loads all PCD perception parameters
 *        from the ROS 2 parameter server.
 *
 * Both PclFilter and PclClustering inherit from this class so that every
 * tunable value lives in exactly one place (config/pcd_params.yaml) and is
 * never duplicated in code.
 */

#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace pcd
{

/**
 * @class PcdConstants
 * @brief Mixin base that declares and caches all perception parameters.
 *
 * Inherit from this class alongside rclcpp::Node (via CRTP or multiple
 * inheritance) and call load_constants() from your constructor *after*
 * declare_parameters() has been invoked.
 *
 * Example:
 * @code
 *   class PclFilter : public rclcpp::Node, public pcd::PcdConstants
 *   {
 *   public:
 *     PclFilter() : rclcpp::Node("pcl_filter"), PcdConstants(this) {}
 *   };
 * @endcode
 */
class PcdConstants
{
  public:
    /**
     * @brief Construct the constants mixin.
     * @param node  Pointer to the owning rclcpp::Node.  Parameters are
     *              declared and read from this node's parameter server.
     */
    explicit PcdConstants(rclcpp::Node *node)
    {
        // ── Distance pass-through filter ────────────────────────────────────
        node->declare_parameter("min_distance", 0.5);
        node->declare_parameter("max_distance", 30.0);

        // ── Water-surface / height rejection ────────────────────────────────
        node->declare_parameter("water_z_min", -0.5);
        node->declare_parameter("water_z_max", 10.0);

        // ── Voxel-grid down-sampling ─────────────────────────────────────────
        node->declare_parameter("voxel_leaf_size", 0.1);

        // ── Euclidean clustering ─────────────────────────────────────────────
        node->declare_parameter("cluster_tolerance", 0.5);
        node->declare_parameter("cluster_min_points", 20);
        node->declare_parameter("cluster_max_points", 25000);

        // ── I/O ──────────────────────────────────────────────────────────────
        node->declare_parameter<std::string>("input_topic", "/velodyne_points");

        // Read them all back into member variables.
        min_distance_ = node->get_parameter("min_distance").as_double();
        max_distance_ = node->get_parameter("max_distance").as_double();
        water_z_min_ = node->get_parameter("water_z_min").as_double();
        water_z_max_ = node->get_parameter("water_z_max").as_double();
        voxel_leaf_size_ = node->get_parameter("voxel_leaf_size").as_double();
        cluster_tolerance_ = node->get_parameter("cluster_tolerance").as_double();
        cluster_min_points_ = node->get_parameter("cluster_min_points").as_int();
        cluster_max_points_ = node->get_parameter("cluster_max_points").as_int();
        input_topic_ = node->get_parameter("input_topic").as_string();
    }

  protected:
    // ── Distance filter ──────────────────────────────────────────────────────
    /// Minimum radial distance [m]. Points closer than this are discarded.
    double min_distance_{ 0.5 };
    /// Maximum radial distance [m]. Points farther than this are discarded.
    double max_distance_{ 30.0 };

    // ── Water-surface rejection ──────────────────────────────────────────────
    /// Lower Z bound [m]. Points below this are treated as water returns.
    double water_z_min_{ -0.5 };
    /// Upper Z bound [m]. Points above this are treated as sky / mast noise.
    double water_z_max_{ 10.0 };

    // ── Voxel-grid down-sampling ─────────────────────────────────────────────
    /// Voxel leaf size [m]. Set to 0.0 to disable down-sampling.
    double voxel_leaf_size_{ 0.1 };

    // ── Euclidean cluster extraction ─────────────────────────────────────────
    /// Max distance [m] between two points to be considered neighbours.
    double cluster_tolerance_{ 0.5 };
    /// Minimum number of points for a cluster to be kept.
    int cluster_min_points_{ 20 };
    /// Maximum number of points allowed in a single cluster.
    int cluster_max_points_{ 25000 };

    // ── Topics ───────────────────────────────────────────────────────────────
    /// Input PointCloud2 topic name.
    std::string input_topic_{ "/velodyne_points" };
};

}  // namespace pcd
