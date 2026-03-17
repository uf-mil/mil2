#pragma once

// #include <mil_bounds/BoundsConfig.h>
#include <pcl/filters/passthrough.h>

#include "nav_msgs/msg/odometry.hpp"
// #include <roboteq_msgs/Command.h>
#include <boost/circular_buffer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/convert.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.hpp>

// Uncomment later for more than vision stuff
// #include "input_cloud_filter.hpp"
// #include "marker_manager.hpp"
// #include "mil_msgs/msg/object_db_query.hpp"
// #include "object_associator.hpp"
// #include "object_detector.hpp"
// #include "object_map.hpp"
// #include "ogrid_manager.hpp"

#include "pcodar_types.hpp"
#include "persistent_cloud_filter.hpp"
#include "point_cloud_builder.hpp"

namespace pcodar
{
/**
 * Base class for a class implementing the object detection needed for the PCODAR node.
 * This can be fulfilled by processesing LIDAR pointclouds (like in @pcodar::Node) or using
 * simulated ground truth (like in pcodar_gazebo).
 */
class NodeBase : public rclcpp::Node
{
  public:
    /// Create a NodeBase in the namespace of nh

    /// creating new NodeBase using ROS2
    NodeBase(std::string const& node_name);

    /// Initialize ROS communication
    virtual void initialize();
    /// Update markers, ogrid, and publish the internal object map to ROS interfaces. Call after updating objects.
    void UpdateObjects();

  protected:
    /// Process a database query ROS service
    // bool DBQuery_cb(mil_msgs::ObjectDBQuery::Request& req, mil_msgs::ObjectDBQuery::Response& res);

    /// Reset PCODAR
    virtual bool Reset(std::shared_ptr<std_srvs::srv::Trigger::Request> const req,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    /// Transform
    /// Rostime not used for ROS2
    bool transform_to_global(std::string const& frame, rclcpp::Time const& time, Eigen::Affine3d& out);
    /// Transform a pointcloud ROS message into a PCL pointcloud in the global frame
    bool transform_point_cloud(sensor_msgs::msg::PointCloud2 const& pcloud2, point_cloud_i& out);
    // virtual bool bounds_update_cb(mil_bounds::BoundsConfig const& config);

    virtual void ConfigCallback(Config const& config, uint32_t level);

  public:
    // std::shared_ptr<ObjectMap> objects_;

  protected:
    // ros::NodeHandle nh_;
    // dynamic_reconfigure::Client<mil_bounds::BoundsConfig> bounds_client_;
    // dynamic_reconfigure::Server<Config> config_server_;

    // ros::ServiceServer modify_classification_service_;
    // ros::ServiceServer reset_service_;

    // std::string global_frame_;

    // tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tf_listener;

    Config config_;
    std::string global_frame_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Publishers
    // ros::Publisher pub_objects_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_objects_;

    point_cloud_ptr bounds_;

    // Visualization

    // Uncomment later
    // MarkerManager marker_manager_;
    // OgridManager ogrid_manager_;

    // Intensity filter
    double intensity_filter_min_intensity;
    double intensity_filter_max_intensity;
};

class Node : public NodeBase
{
  public:
    // making a new node now
    Node();

    void velodyne_cb(sensor_msgs::msg::PointCloud2::SharedPtr const pcloud);

    void initialize() override;

  private:
    // bool bounds_update_cb(mil_bounds::BoundsConfig const& config) override;
    void ConfigCallback(Config const& config, uint32_t level) override;
    void update_config(Config const& config);
    /// Reset PCODAR
    bool Reset(std::shared_ptr<std_srvs::srv::Trigger::Request> const req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) override;

  private:
    // ros::Publisher pub_pcl_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_;

    // Subscriber
    // ros::Subscriber pc_sub;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;  // for reset requests

    // Model (It eventually will be object tracker, but for now just detections)
    // InputCloudFilter input_cloud_filter_;
    // PersistentCloudFilter persistent_cloud_filter_;
    // PointCloudCircularBuffer persistent_cloud_builder_;
    // ObjectDetector detector_;

    // uncomment later
    // Associator ass;
};

}  // namespace pcodar
