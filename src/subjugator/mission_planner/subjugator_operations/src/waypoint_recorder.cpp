#include <cmath>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

class WaypointRecorder : public rclcpp::Node
{
  public:
    WaypointRecorder()
      : Node("waypoint_recorder")
      , spacing_m_(declare_parameter<double>("spacing_m", 0.5))
      , odom_topic_(declare_parameter<std::string>("odom_topic", "/odometry/filtered"))
      , path_topic_(declare_parameter<std::string>("path_topic", "/recorded_path"))
      , have_last_(false)
    {
        using std::placeholders::_1;

        path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(1).transient_local());

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(50),
                                                                 std::bind(&WaypointRecorder::odomCb, this, _1));

        RCLCPP_INFO(get_logger(), "WaypointRecorder started. odom_topic='%s', spacing=%.2fm, path_topic='%s'",
                    odom_topic_.c_str(), spacing_m_, path_topic_.c_str());
    }

  private:
    static double dist2D(geometry_msgs::msg::Point const &a, geometry_msgs::msg::Point const &b)
    {
        double const dx = a.x - b.x;
        double const dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void odomCb(nav_msgs::msg::Odometry::SharedPtr const msg)
    {
        // Build a PoseStamped waypoint in the same frame/time as the odometry
        geometry_msgs::msg::PoseStamped wp;
        wp.header = msg->header;   // keeps frame_id + stamp
        wp.pose = msg->pose.pose;  // position + orientation from fused odom

        std::scoped_lock<std::mutex> lock(mutex_);

        if (!have_last_)
        {
            // First waypoint = starting pose
            pushWaypointLocked(wp);
            last_wp_ = wp;
            have_last_ = true;
            return;
        }

        // Record only when we moved >= spacing_m_ from last recorded waypoint
        double const d = dist2D(wp.pose.position, last_wp_.pose.position);
        if (d >= spacing_m_)
        {
            pushWaypointLocked(wp);
            last_wp_ = wp;
        }
    }

    void pushWaypointLocked(geometry_msgs::msg::PoseStamped const &wp)
    {
        waypoints_.push_back(wp);

        // Publish as nav_msgs/Path (handy for RViz + downstream)
        nav_msgs::msg::Path path;
        path.header = wp.header;  // frame_id matches odom frame
        path.poses = waypoints_;

        path_pub_->publish(path);

        RCLCPP_DEBUG(get_logger(), "Recorded waypoint #%zu at (%.2f, %.2f, %.2f)", waypoints_.size(),
                     wp.pose.position.x, wp.pose.position.y, wp.pose.position.z);
    }

  private:
    double spacing_m_;
    std::string odom_topic_;
    std::string path_topic_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    std::mutex mutex_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    geometry_msgs::msg::PoseStamped last_wp_;
    bool have_last_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointRecorder>());
    rclcpp::shutdown();
    return 0;
}
