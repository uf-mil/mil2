#include "pcl_stack/input_cloud_filter.hpp"

namespace pcodar
{
InputCloudFilter::InputCloudFilter()
{
    bounds_filter_.setDim(2);
    bounds_filter_.setCropOutside(true);

    robot_filter_.setNegative(true);
}

void InputCloudFilter::update_config(Config const& config)
{
    Eigen::Vector4f min(-static_cast<float>(config.robot_footprint_half_length_m),
                        -static_cast<float>(config.robot_footprint_half_width_m),
                        static_cast<float>(config.robot_footprint_min_z_m), 1.f);
    Eigen::Vector4f max(static_cast<float>(config.robot_footprint_half_length_m),
                        static_cast<float>(config.robot_footprint_half_width_m),
                        static_cast<float>(config.robot_footprint_max_z_m), 1.f);
    robot_filter_.setMin(min);
    robot_filter_.setMax(max);
}

void InputCloudFilter::filter(point_cloud_const_ptr in, point_cloud& pc)
{
    point_cloud_const_ptr hull_input = in;
    point_cloud_ptr tmp;
    if (bounds_configured_)
    {
        tmp = std::make_shared<point_cloud>();
        bounds_filter_.setInputCloud(in);
        bounds_filter_.filter(*tmp);
        hull_input = tmp;
    }

    robot_filter_.setInputCloud(hull_input);
    robot_filter_.filter(pc);
}

void InputCloudFilter::set_bounds(point_cloud_ptr bounds)
{
    pcl::Vertices indices;
    indices.vertices.reserve(bounds->size());
    for (size_t i = 0; i < bounds->size(); ++i)
        indices.vertices.push_back(i);
    std::vector<pcl::Vertices> hull = { indices };

    bounds_filter_.setHullCloud(bounds);
    bounds_filter_.setHullIndices(hull);
    bounds_configured_ = (bounds != nullptr && bounds->size() >= 3);
}

void InputCloudFilter::set_robot_footprint(Eigen::Vector4f const& min, Eigen::Vector4f const& max)
{
    robot_filter_.setMin(min);
    robot_filter_.setMax(max);
}

void InputCloudFilter::set_robot_pose(Eigen::Affine3d const& transform)
{
    Eigen::Affine3f transform_float = transform.inverse().cast<float>();
    robot_filter_.setTransform(transform_float);
}

}  // namespace pcodar
