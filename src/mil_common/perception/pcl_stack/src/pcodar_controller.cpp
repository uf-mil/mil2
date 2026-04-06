#include "pcl_stack/pcodar_controller.hpp"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <functional>

namespace pcodar
{

// declared as a ROS1 nodeHandler,
// NodeBase::NodeBase(ros::NodeHandle _nh)
//   : nh_(_nh)
//   , bounds_client_("/bounds_server", std::bind(&NodeBase::bounds_update_cb, this, std::placeholders::_1))
//   , tf_listener(tf_buffer_, nh_)
//   , global_frame_("enu")
//   , config_server_(_nh)
//   , intensity_filter_min_intensity(10)
//   , intensity_filter_max_intensity(100)
//   , objects_(std::make_shared<ObjectMap>())
// {
//     config_server_.setCallback(
//         std::bind(&NodeBase::ConfigCallback, this, std::placeholders::_1, std::placeholders::_2));
// }

// Inherits directly from rclcpp in header
NodeBase::NodeBase(std::string const& node_name)
  : rclcpp::Node(node_name)
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , global_frame_("enu")
  , intensity_filter_min_intensity(10)
  , intensity_filter_max_intensity(100)
{
}

void NodeBase::ConfigCallback(Config const& config, uint32_t level)
{
    // don't need this for now, will have to change later

    // if (!level || level & 16)
    //     ogrid_manager_.update_config(config);
}

void NodeBase::UpdateObjects()
{
    // uncomment later

    // ogrid_manager_.update_ogrid(*objects_);
    // auto objects_msg = objects_->to_msg();
    // marker_manager_.update_markers();
    // pub_objects_.publish(objects_msg);
}

void NodeBase::initialize()
{
    // marker_manager_.initialize(nh_, objects_);
    // ogrid_manager_.initialize(nh_);

    // modify_classification_service_ = nh_.advertiseService("/database/requests", &NodeBase::DBQuery_cb, this);
    // reset_service_ = nh_.advertiseService("reset", &NodeBase::Reset, this);

    // // Publish PerceptionObjects
    // pub_objects_ = nh_.advertise<mil_msgs::PerceptionObjectArray>("objects", 1);
}

// bool NodeBase::transform_to_global(std::string const& frame, ros::Time const& time, Eigen::Affine3d& out,
//                                    ros::Duration timeout)
// {
//     geometry_msgs::TransformStamped transform;
//     try
//     {
//         transform = tf_buffer_.lookupTransform("enu", frame, time, timeout);
//     }
//     catch (tf2::TransformException& ex)
//     {
//         ROS_ERROR("%s", ex.what());
//         return false;
//     }
//     out = tf2::transformToEigen(transform);
//     return true;
// }

bool NodeBase::transform_to_global(std::string const& frame, rclcpp::Time const& time, Eigen::Affine3d& out)
{
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = tf_buffer_.lookupTransform(global_frame_, frame, time, tf2::durationFromSec(1.0));
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return false;
    }
    out = tf2::transformToEigen(transform);
    return true;
}

bool NodeBase::Reset(std::shared_ptr<std_srvs::srv::Trigger::Request> const req,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    // marker_manager_.reset();
    // objects_->objects_.clear();
    return true;
}

// bool NodeBase::bounds_update_cb(mil_bounds::BoundsConfig const& config)
// {
//     Eigen::Affine3d transform;
//     if (!transform_to_global(config.frame, ros::Time::now(), transform, ros::Duration(10)))
//         return false;

//     bounds_ = boost::make_shared<point_cloud>();
//     bounds_->push_back(point_t(config.x1, config.y1, config.z1));
//     bounds_->push_back(point_t(config.x2, config.y2, config.z2));
//     bounds_->push_back(point_t(config.x3, config.y3, config.z3));
//     bounds_->push_back(point_t(config.x4, config.y4, config.z4));

//     pcl::transformPointCloud(*bounds_, *bounds_, transform);
//     ogrid_manager_.set_bounds(bounds_);

//     return true;
// }

// bool NodeBase::DBQuery_cb(mil_msgs::ObjectDBQuery::Request& req, mil_msgs::ObjectDBQuery::Response& res)
// {
//     // return objects_->DatabaseQuery(req, res);
// }

bool NodeBase::transform_point_cloud(sensor_msgs::msg::PointCloud2 const& pc_msg, point_cloud_i& out)
{
    Eigen::Affine3d transform;
    if (!transform_to_global(pc_msg.header.frame_id, pc_msg.header.stamp, transform))
        return false;

    // Change pc_msg to a new point cloud
    point_cloud_i pcloud;
    pcl::fromROSMsg(pc_msg, pcloud);

    out.clear();
    // pcl::transformPointCloud expects Eigen::Affine3f; tf2 gives Affine3d
    pcl::transformPointCloud(pcloud, out, transform.cast<float>());
    return true;
}

// Node::Node(ros::NodeHandle _nh) : NodeBase(_nh)
// {
//     config_server_.setCallback(std::bind(&Node::ConfigCallback, this, std::placeholders::_1, std::placeholders::_2));

//     // TODO: pull from params (currently misleading)
//     // Currently based on WAMv in VRX, inflated x 1.2 for safety factor
//     double const HALF_LENGTH = 2.739625 * 1.2;
//     double const HALF_WIDTH = 2.02589 * 1.2;
//     double const BOTTOM = -5.;
//     double const TOP = 5.;
//     Eigen::Vector4f min(-HALF_LENGTH, -HALF_WIDTH, BOTTOM, 1.);
//     Eigen::Vector4f max(HALF_LENGTH, HALF_WIDTH, TOP, 1.);

//     // Give the filter the footprint of the robot to remove from pointcloud
//     input_cloud_filter_.set_robot_footprint(min, max);
// }

Node::Node() : NodeBase("pcl_stack_node")
{
    // pull all params
    this->declare_parameter("accumulator_number_persistant_clouds", config_.accumulator_number_persistant_clouds);
    this->declare_parameter("persistant_cloud_filter_radius", config_.persistant_cloud_filter_radius);
    this->declare_parameter("persistant_cloud_filter_min_neighbors", config_.persistant_cloud_filter_min_neighbors);
    this->declare_parameter("cluster_tolerance_m", config_.cluster_tolerance_m);
    this->declare_parameter("cluster_min_points", config_.cluster_min_points);
    this->declare_parameter("intensity_filter_min_intensity", config_.intensity_filter_min_intensity);
    this->declare_parameter("intensity_filter_max_intensity", config_.intensity_filter_max_intensity);
    this->declare_parameter("robot_footprint_half_length_m", config_.robot_footprint_half_length_m);
    this->declare_parameter("robot_footprint_half_width_m", config_.robot_footprint_half_width_m);
    this->declare_parameter("robot_footprint_min_z_m", config_.robot_footprint_min_z_m);
    this->declare_parameter("robot_footprint_max_z_m", config_.robot_footprint_max_z_m);

    // Read declared params into config_ (declare_parameter does not assign from YAML automatically)
    (void)this->get_parameter("accumulator_number_persistant_clouds", config_.accumulator_number_persistant_clouds);
    (void)this->get_parameter("persistant_cloud_filter_radius", config_.persistant_cloud_filter_radius);
    (void)this->get_parameter("persistant_cloud_filter_min_neighbors", config_.persistant_cloud_filter_min_neighbors);
    (void)this->get_parameter("cluster_tolerance_m", config_.cluster_tolerance_m);
    (void)this->get_parameter("cluster_min_points", config_.cluster_min_points);
    (void)this->get_parameter("intensity_filter_min_intensity", config_.intensity_filter_min_intensity);
    (void)this->get_parameter("intensity_filter_max_intensity", config_.intensity_filter_max_intensity);
    (void)this->get_parameter("robot_footprint_half_length_m", config_.robot_footprint_half_length_m);
    (void)this->get_parameter("robot_footprint_half_width_m", config_.robot_footprint_half_width_m);
    (void)this->get_parameter("robot_footprint_min_z_m", config_.robot_footprint_min_z_m);
    (void)this->get_parameter("robot_footprint_max_z_m", config_.robot_footprint_max_z_m);

    this->declare_parameter("use_input_cloud_filter", use_input_cloud_filter_);
    this->declare_parameter("use_object_detector", use_object_detector_);
    this->declare_parameter("robot_frame", robot_frame_);
    this->declare_parameter<std::string>("global_frame", global_frame_);
    use_input_cloud_filter_ = this->get_parameter("use_input_cloud_filter").as_bool();
    use_object_detector_ = this->get_parameter("use_object_detector").as_bool();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    global_frame_ = this->get_parameter("global_frame").as_string();

    // Wire up components from config_
    persistent_cloud_builder_.update_config(config_);
    persistent_cloud_filter_.update_config(config_);
    input_cloud_filter_.update_config(config_);
    detector_.update_config(config_);
}

void Node::update_config(Config const& config)
{
    // Full snapshot (e.g. future dynamic reconfigure); constructor loads config_ from parameters separately
    config_ = config;
    this->intensity_filter_min_intensity = config_.intensity_filter_min_intensity;
    this->intensity_filter_max_intensity = config_.intensity_filter_max_intensity;
    persistent_cloud_builder_.update_config(config_);
    persistent_cloud_filter_.update_config(config_);
    input_cloud_filter_.update_config(config_);
    detector_.update_config(config_);
}

void Node::ConfigCallback(Config const& config, uint32_t level)
{
    NodeBase::ConfigCallback(config, level);
    if (!level || level & 1)
        // persistent_cloud_builder_.update_config(config);
        if (!level || level & 2)
            // persistent_cloud_filter_.update_config(config);
            if (!level || level & 4)
                // detector_.update_config(config);
                if (!level || level & 8)
                    // ass.update_config(config);
                    if (!level || level & 32)
                        this->update_config(config);
}

void Node::initialize()
{
    NodeBase::initialize();

    // Subscribe pointcloud
    // pc_sub = nh_.subscribe("/velodyne_points", 1, &Node::velodyne_cb, this);

    // // Publish occupancy grid and visualization markers
    // pub_pcl_ = nh_.advertise<point_cloud>("persist_pcl", 1);

    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", rclcpp::QoS(1), std::bind(&Node::velodyne_cb, this, std::placeholders::_1));

    pub_pcl_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("persist_pcl", rclcpp::QoS(1));
}

bool Node::Reset(std::shared_ptr<std_srvs::srv::Trigger::Request> const req,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (!NodeBase::Reset(req, res))
        return false;
    persistent_cloud_builder_.clear();
    res->success = true;
    return true;
}

void Node::velodyne_cb(sensor_msgs::msg::PointCloud2::SharedPtr const pcloud)
{
    point_cloud_i_ptr pc = std::make_shared<point_cloud_i>();
    pcl::fromROSMsg(*pcloud, *pc);

    // Intensity filter
    pcl::PassThrough<pointi_t> intensity_filter;
    intensity_filter.setInputCloud(pc);
    intensity_filter.setFilterFieldName("intensity");
    intensity_filter.setFilterLimits(this->intensity_filter_min_intensity, this->intensity_filter_max_intensity);
    point_cloud_ptr pc_without_i = std::make_shared<point_cloud>();
    point_cloud_i_ptr pc_i_filtered = std::make_shared<point_cloud_i>();
    intensity_filter.filter(*pc_i_filtered);

    pc_without_i->points.resize(pc_i_filtered->size());
    for (size_t i = 0; i < pc_i_filtered->points.size(); i++)
    {
        pc_without_i->points[i].x = pc_i_filtered->points[i].x;
        pc_without_i->points[i].y = pc_i_filtered->points[i].y;
        pc_without_i->points[i].z = pc_i_filtered->points[i].z;
    }

    if (pc_without_i->empty())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Filtered pointcloud had no points. Consider changing filter parameters.");
        return;
    }

    std::string output_frame_id = pcloud->header.frame_id.empty() ? "velodyne" : pcloud->header.frame_id;
    point_cloud_ptr cloud_for_buffer = pc_without_i;

    if (use_input_cloud_filter_)
    {
        Eigen::Affine3d T_enu_sensor;
        if (!transform_to_global(pcloud->header.frame_id, pcloud->header.stamp, T_enu_sensor))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "input_cloud_filter: TF %s -> %s failed; skipping scan",
                                 pcloud->header.frame_id.c_str(), global_frame_.c_str());
            return;
        }

        point_cloud global_xyz;
        pcl::transformPointCloud(*pc_without_i, global_xyz, T_enu_sensor.cast<float>());
        point_cloud_ptr const gptr = std::make_shared<point_cloud>(global_xyz);

        geometry_msgs::msg::TransformStamped tf_enu_robot;
        try
        {
            tf_enu_robot = tf_buffer_.lookupTransform(global_frame_, robot_frame_, pcloud->header.stamp,
                                                      tf2::durationFromSec(1.0));
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "input_cloud_filter: TF %s -> %s failed (%s); skipping scan", robot_frame_.c_str(),
                                 global_frame_.c_str(), ex.what());
            return;
        }

        Eigen::Affine3d const T_enu_robot = tf2::transformToEigen(tf_enu_robot);
        input_cloud_filter_.set_robot_pose(T_enu_robot);

        point_cloud footprint_filtered;
        input_cloud_filter_.filter(gptr, footprint_filtered);
        if (footprint_filtered.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "input_cloud_filter removed all points (check footprint / bounds).");
            return;
        }

        cloud_for_buffer = std::make_shared<point_cloud>(footprint_filtered);
        output_frame_id = global_frame_;
    }

    persistent_cloud_builder_.add_point_cloud(cloud_for_buffer);
    point_cloud_ptr const mega_cloud = persistent_cloud_builder_.get_point_cloud();
    if (!mega_cloud)
        return;

    point_cloud filtered_mega;
    persistent_cloud_filter_.filter(mega_cloud, filtered_mega);
    if (filtered_mega.empty())
        return;

    if (use_object_detector_)
    {
        point_cloud_ptr const mega_ptr = std::make_shared<point_cloud>(filtered_mega);
        clusters_t const clusters = detector_.get_clusters(mega_ptr);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "object_detector: %zu clusters",
                             clusters.size());
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(filtered_mega, cloud_msg);
    cloud_msg.header.frame_id = output_frame_id;
    cloud_msg.header.stamp = pcloud->header.stamp;
    pub_pcl_->publish(cloud_msg);

    UpdateObjects();
}

// bool Node::bounds_update_cb(mil_bounds::BoundsConfig const& config)
// {
//     if (!NodeBase::bounds_update_cb(config))
//         return false;
//     input_cloud_filter_.set_bounds(bounds_);
//     return true;
// }

}  // namespace pcodar
