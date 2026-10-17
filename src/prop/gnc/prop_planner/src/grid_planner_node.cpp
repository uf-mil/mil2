#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "prop_planner/grid_planner.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace prop_planner
{
namespace
{

/// The part of a route still ahead: from the boat, joining at the nearest leg.
std::vector<Point> remaining(std::vector<Point> const& route, Point boat)
{
    std::size_t best = 1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < route.size(); ++i)
    {
        double const d = distance_to_segment(boat, route[i - 1], route[i]);
        if (d < best_distance)
        {
            best_distance = d;
            best = i;
        }
    }
    std::vector<Point> ahead{ boat };
    ahead.insert(ahead.end(), route.begin() + static_cast<std::ptrdiff_t>(best), route.end());
    return ahead;
}

}  // namespace

// Routes the boat through the evidence grid.
//
//   in   occupancy_grid  nav_msgs/OccupancyGrid, from the occupancy_grid node
//        odometry        nav_msgs/Odometry, must be published in map_frame
//        goal_pose       geometry_msgs/PoseStamped, RViz's "2D Goal Pose"
//   srv  ~/enable        std_srvs/SetBool; enabling clears the grid and the goal
//   out  plan            nav_msgs/Path in map_frame, latched, for guidance
//        costmap         nav_msgs/OccupancyGrid, what the route is steering by
//
// A route is kept until it is blocked or a fresh one is replan_gain cheaper.
// Guidance restarts its leg on every plan it receives, so a route that merely
// shifted a little is not worth sending.
class GridPlannerNode : public rclcpp::Node
{
  public:
    GridPlannerNode()
      : Node("planner")
      , planner_({ static_cast<int>(declare_parameter("occupied", 60)), declare_parameter("lethal_radius", 0.6),
                   declare_parameter("clearance", 2.0), declare_parameter("clearance_weight", 2.0) })
    {
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        replan_gain_ = declare_parameter("replan_gain", 0.15);
        enabled_ = declare_parameter("start_enabled", true);
        double const rate = declare_parameter("rate", 2.0);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::QoS latched(1);
        latched.transient_local();

        grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "occupancy_grid", latched,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr const msg) { grid_callback(*msg); });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odometry", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr const msg)
            {
                if (msg->header.frame_id != map_frame_)
                {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                          "odometry is in %s but the map is in %s; remap odometry or set map_frame",
                                          msg->header.frame_id.c_str(), map_frame_.c_str());
                    return;
                }
                position_ = { msg->pose.pose.position.x, msg->pose.pose.position.y };
                located_ = true;
            });

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr const msg) { goal_callback(*msg); });

        plan_pub_ = create_publisher<nav_msgs::msg::Path>("plan", latched);
        costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", rclcpp::QoS(1));
        clear_client_ = create_client<std_srvs::srv::Trigger>("occupancy_grid/clear");

        enable_srv_ = create_service<std_srvs::srv::SetBool>(
            "~/enable",
            [this](std_srvs::srv::SetBool::Request::SharedPtr const request,
                   std_srvs::srv::SetBool::Response::SharedPtr const response)
            {
                // A task starts from what the boat can see, not what the last one left.
                if (request->data && !enabled_)
                {
                    has_goal_ = false;
                    published_.clear();
                    if (clear_client_->service_is_ready())
                    {
                        clear_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
                    }
                }
                enabled_ = request->data;
                response->success = true;
                response->message = enabled_ ? "planning, grid cleared" : "stopped";
                RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
            });

        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { replan(); });

        RCLCPP_INFO(get_logger(), "grid planner %s - send a goal on goal_pose, or use RViz's 2D Goal Pose",
                    enabled_ ? "running" : "idle, waiting on ~/enable");
    }

  private:
    void grid_callback(nav_msgs::msg::OccupancyGrid const& msg)
    {
        if (msg.header.frame_id != map_frame_)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "occupancy_grid is in %s but the map is in %s",
                                  msg.header.frame_id.c_str(), map_frame_.c_str());
            return;
        }
        grid_.width = static_cast<int>(msg.info.width);
        grid_.height = static_cast<int>(msg.info.height);
        grid_.resolution = msg.info.resolution;
        grid_.origin = { msg.info.origin.position.x, msg.info.origin.position.y };
        grid_.cells = msg.data;
        has_grid_ = true;
    }

    void goal_callback(geometry_msgs::msg::PoseStamped const& msg)
    {
        if (!enabled_)
        {
            return;
        }
        // RViz stamps the goal in its fixed frame, which need not be the map's.
        geometry_msgs::msg::PoseStamped goal;
        try
        {
            goal = tf_buffer_->transform(msg, map_frame_, tf2::durationFromSec(0.2));
        }
        catch (tf2::TransformException const& ex)
        {
            RCLCPP_WARN(get_logger(), "goal in %s cannot be brought into %s, ignoring it: %s",
                        msg.header.frame_id.c_str(), map_frame_.c_str(), ex.what());
            return;
        }
        goal_ = { goal.pose.position.x, goal.pose.position.y };
        goal_orientation_ = goal.pose.orientation;
        has_goal_ = true;
        published_.clear();
        RCLCPP_INFO(get_logger(), "new goal at (%.1f, %.1f) in %s", goal_.x, goal_.y, map_frame_.c_str());
    }

    void replan()
    {
        if (!enabled_ || !has_grid_ || !located_ || !has_goal_)
        {
            return;
        }

        Costmap const map = planner_.costmap(grid_, position_, goal_);
        publish_costmap(map);

        std::vector<Point> const route = planner_.plan(map, position_, goal_);
        if (route.empty())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no route to (%.1f, %.1f); holding the last plan",
                                 goal_.x, goal_.y);
            return;
        }
        if (!published_.empty() &&
            route_cost(map, remaining(published_, position_)) <= (1.0 + replan_gain_) * route_cost(map, route))
        {
            return;
        }

        publish_plan(route);
        published_ = route;
        RCLCPP_INFO(get_logger(), "planned %zu waypoints to (%.1f, %.1f)", route.size() - 1, goal_.x, goal_.y);
    }

    void publish_plan(std::vector<Point> const& route) const
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = map_frame_;
        path.header.stamp = now();
        for (std::size_t i = 1; i < route.size(); ++i)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = route[i].x;
            pose.pose.position.y = route[i].y;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        // Guidance reads only the last pose's orientation, as the heading to finish on.
        path.poses.back().pose.orientation = goal_orientation_;
        plan_pub_->publish(path);
    }

    void publish_costmap(Costmap const& map) const
    {
        nav_msgs::msg::OccupancyGrid out;
        out.header.frame_id = map_frame_;
        out.header.stamp = now();
        out.info.resolution = static_cast<float>(grid_.resolution);
        out.info.width = static_cast<uint32_t>(grid_.width);
        out.info.height = static_cast<uint32_t>(grid_.height);
        out.info.origin.position.x = grid_.origin.x;
        out.info.origin.position.y = grid_.origin.y;
        out.info.origin.orientation.w = 1.0;

        double const weight = std::max(get_parameter("clearance_weight").as_double(), 1e-9);
        out.data.reserve(map.cost.size());
        for (float const cost : map.cost)
        {
            out.data.push_back(cost >= Costmap::kLethal ? 100 :
                                                          static_cast<int8_t>(std::min(99.0, 99.0 * cost / weight)));
        }
        costmap_pub_->publish(out);
    }

    std::string map_frame_;
    double replan_gain_;
    bool enabled_{ true };

    GridPlanner planner_;
    Grid grid_;
    bool has_grid_{ false };

    Point position_{};
    bool located_{ false };
    Point goal_{};
    geometry_msgs::msg::Quaternion goal_orientation_;
    bool has_goal_{ false };
    std::vector<Point> published_;  ///< the route guidance is on, boat's position first

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_client_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace prop_planner

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<prop_planner::GridPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
