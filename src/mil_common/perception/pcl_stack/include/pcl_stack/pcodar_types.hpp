#pragma once

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
// #include <pcl_stack/PCODARConfig.h>

#include "mil_msgs/msg/perception_object.hpp"

namespace pcodar
{

struct Config
{
    // May need to be re-tuned?

    // Point cloud builder - How many consectuive lidar scans to stakc before processing them together at once.
    int accumulator_number_persistant_clouds = 10;

    // Filter - radius (meters) for utlier removal
    double persistant_cloud_filter_radius = 0.5;
    int persistant_cloud_filter_min_neighbors = 20;

    // Clusterer -max distance between points to be considered as cluster
    double cluster_tolerance_m = 4.4;
    int cluster_min_points = 2;  // min points to be considered a cluster

    // Associator
    double associator_max_distance = 25000.0;  // to match a new obj with existing obj
    bool associator_forget_unseen = false;     // if true, removes obj from database when no longer visible.

    // Ogrid
    double ogrid_height_meters = 1000.0;  // size of occupancy grid.
    double ogrid_width_meters = 1000.0;
    double ogrid_resolution_meters_per_cell = 0.3;
    double ogrid_inflation_meters = 2.0;  // inflates for path planning marigin

    // Intensity filter
    double intensity_filter_min_intensity = 10.0;
    double intensity_filter_max_intensity = 100.0;
};

// /// Alias for the dynamic reconfigure object used for PCODAR configuration
// using Config = point_cloud_object_detection_and_recognition::PCODARConfig;

/// Alias for type of pointcloud used internally
using point_t = pcl::PointXYZ;
using pointi_t = pcl::PointXYZI;
/// Pointcloud of point_t
using point_cloud = pcl::PointCloud<point_t>;
using point_cloud_i = pcl::PointCloud<pointi_t>;
/// Pointer to PCODAR's pointclouds
using point_cloud_ptr = point_cloud::Ptr;
using point_cloud_i_ptr = point_cloud_i::Ptr;
/// Constant pointer to PCODAR's pointclouds
using point_cloud_const_ptr = point_cloud::ConstPtr;
using KdTree = pcl::search::KdTree<point_t>;
using KdTreePtr = KdTree::Ptr;

/// Clusters used in object detection
using cluster_t = pcl::PointIndices;
/// Vector of clusters
using clusters_t = std::vector<cluster_t>;

}  // namespace pcodar
