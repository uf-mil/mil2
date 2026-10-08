/**
 * @file scan_to_cloud_node.cpp
 * @brief Republishes a 2D laser scan as a 3D point cloud.
 *
 * SIMULATOR SUPPORT ONLY. The simulated boat carries a flat single-ring
 * scanner publishing sensor_msgs/LaserScan, but the pcd filtering and
 * clustering pipeline consumes sensor_msgs/PointCloud2. The real boat's lidar
 * publishes PointCloud2 directly, so this node is not run there.
 *
 *   in   scan    sensor_msgs/LaserScan
 *   out  points  sensor_msgs/PointCloud2, same frame and timestamp
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ScanToCloud : public rclcpp::Node
{
  public:
    ScanToCloud() : rclcpp::Node("scan_to_cloud")
    {
        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());

        subscription_ =
            create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(),
                                                             [this](sensor_msgs::msg::LaserScan::SharedPtr const msg)
                                                             {
                                                                 sensor_msgs::msg::PointCloud2 cloud;
                                                                 projector_.projectLaser(*msg, cloud);
                                                                 // projectLaser carries the scan's frame and stamp
                                                                 // across, which is what the position chain needs to
                                                                 // place the points later.
                                                                 publisher_->publish(cloud);
                                                             });

        RCLCPP_INFO(get_logger(), "scan_to_cloud started: LaserScan on 'scan' -> PointCloud2 on 'points'");
    }

  private:
    laser_geometry::LaserProjection projector_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToCloud>());
    rclcpp::shutdown();
    return 0;
}
