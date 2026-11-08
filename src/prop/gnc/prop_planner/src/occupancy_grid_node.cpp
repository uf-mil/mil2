#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "prop_planner/evidence_grid.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace prop_planner
{

// Builds the evidence grid straight from the lidar cloud.
//
//   in   cloud           sensor_msgs/PointCloud2, in any frame TF can take to map_frame
//        imu/data        sensor_msgs/Imu, for the hull's roll and pitch
//   srv  ~/clear         std_srvs/Trigger, forget everything
//   out  occupancy_grid  nav_msgs/OccupancyGrid in map_frame, 0..100 evidence per cell, latched
//
// The raw cloud rather than pcd's filtered one, so what the boat can drive
// through never depends on what clustering chose to keep.
//
// The height band that separates water from obstacle is judged in a level
// frame built from the IMU, not from TF: localization runs two_d_mode, so TF
// carries no roll or pitch, and the few degrees of trim a thrust step puts on
// the hull lift the water 20 m out into the band as a wall. Until an IMU
// message arrives the cloud is taken as level.
class OccupancyGridNode : public rclcpp::Node
{
  public:
    OccupancyGridNode()
      : Node("occupancy_grid")
      , grid_({ declare_parameter("resolution", 0.25), declare_parameter("size", 80.0),
                static_cast<int>(declare_parameter("hit_gain", 25)),
                static_cast<int>(declare_parameter("miss_loss", 10)), declare_parameter("verify_range", 12.0) })
    {
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        min_range_ = declare_parameter("min_range", 1.0);
        max_range_ = declare_parameter("max_range", 12.0);
        z_min_ = declare_parameter("z_min", -0.6);
        z_max_ = declare_parameter("z_max", 2.0);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr const msg) { cloud_callback(*msg); });
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS(),
                                                              [this](sensor_msgs::msg::Imu::SharedPtr const msg)
                                                              { imu_callback(*msg); });

        rclcpp::QoS latched(1);
        latched.transient_local();
        grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", latched);

        clear_srv_ =
            create_service<std_srvs::srv::Trigger>("~/clear",
                                                   [this](std_srvs::srv::Trigger::Request::SharedPtr const,
                                                          std_srvs::srv::Trigger::Response::SharedPtr const response)
                                                   {
                                                       grid_.clear();
                                                       response->success = true;
                                                       response->message = "grid cleared";
                                                   });

        RCLCPP_INFO(get_logger(), "occupancy grid in %s: %.0f m window at %.2f m per cell", map_frame_.c_str(),
                    get_parameter("size").as_double(), get_parameter("resolution").as_double());
    }

  private:
    bool lookup(std::string const& target, std::string const& source, builtin_interfaces::msg::Time const& stamp,
                tf2::Transform& out)
    {
        try
        {
            tf2::fromMsg(tf_buffer_->lookupTransform(target, source, stamp, tf2::durationFromSec(0.1)).transform, out);
            return true;
        }
        catch (tf2::TransformException const& ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no transform %s -> %s: %s", source.c_str(),
                                 target.c_str(), ex.what());
            return false;
        }
    }

    void imu_callback(sensor_msgs::msg::Imu const& msg)
    {
        tf2::Quaternion world_from_imu;
        tf2::fromMsg(msg.orientation, world_from_imu);
        if (world_from_imu.length2() < 0.5)
        {
            return;  // no orientation in this message
        }
        if (!have_mount_)
        {
            tf2::Transform mount;
            if (!lookup(base_frame_, msg.header.frame_id, builtin_interfaces::msg::Time(), mount))
            {
                return;
            }
            base_from_imu_ = mount.getRotation();
            have_mount_ = true;
        }
        // The hull's roll and pitch with its heading taken back out: what
        // turns a point in base_frame into one in a level frame.
        double roll, pitch, yaw;
        tf2::Matrix3x3(world_from_imu * base_from_imu_.inverse()).getRPY(roll, pitch, yaw);
        level_from_base_.setRPY(roll, pitch, 0.0);
    }

    void cloud_callback(sensor_msgs::msg::PointCloud2 const& msg)
    {
        tf2::Transform map_from_lidar;
        tf2::Transform base_from_lidar;
        if (!lookup(map_frame_, msg.header.frame_id, msg.header.stamp, map_from_lidar) ||
            !lookup(base_frame_, msg.header.frame_id, builtin_interfaces::msg::Time(), base_from_lidar))
        {
            return;
        }
        tf2::Transform const level_from_lidar = tf2::Transform(level_from_base_) * base_from_lidar;
        double const lidar_height = level_from_lidar.getOrigin().z();

        hits_.clear();
        for (sensor_msgs::PointCloud2ConstIterator<float> it(msg, "x"); it != it.end(); ++it)
        {
            tf2::Vector3 const p(it[0], it[1], it[2]);
            double const range = p.length();
            if (!std::isfinite(range) || range < min_range_ || range > max_range_)
            {
                continue;
            }
            double const height = (level_from_lidar * p).z() - lidar_height;
            if (height < z_min_ || height > z_max_)
            {
                continue;
            }
            tf2::Vector3 const m = map_from_lidar * p;
            hits_.push_back({ m.x(), m.y() });
        }

        Point const at{ map_from_lidar.getOrigin().x(), map_from_lidar.getOrigin().y() };
        grid_.follow(at);
        grid_.integrate(hits_, at);
        publish(msg.header.stamp);
    }

    void publish(builtin_interfaces::msg::Time const& stamp)
    {
        Grid const& g = grid_.grid();
        nav_msgs::msg::OccupancyGrid out;
        out.header.frame_id = map_frame_;
        out.header.stamp = stamp;
        out.info.resolution = static_cast<float>(g.resolution);
        out.info.width = static_cast<uint32_t>(g.width);
        out.info.height = static_cast<uint32_t>(g.height);
        out.info.origin.position.x = g.origin.x;
        out.info.origin.position.y = g.origin.y;
        out.info.origin.orientation.w = 1.0;
        out.data = g.cells;
        grid_pub_->publish(out);
    }

    std::string map_frame_;
    std::string base_frame_;
    double min_range_;
    double max_range_;
    double z_min_;  ///< m relative to the lidar, level; below is water
    double z_max_;  ///< m relative to the lidar, level; above is nothing the hull can hit

    EvidenceGrid grid_;
    std::vector<Point> hits_;
    tf2::Quaternion level_from_base_{ 0.0, 0.0, 0.0, 1.0 };
    tf2::Quaternion base_from_imu_{ 0.0, 0.0, 0.0, 1.0 };
    bool have_mount_{ false };

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
};

}  // namespace prop_planner

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<prop_planner::OccupancyGridNode>());
    rclcpp::shutdown();
    return 0;
}
