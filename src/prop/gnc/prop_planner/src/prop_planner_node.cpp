#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <vector>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_plan.hpp>

class PropPlanner : public rclcpp::Node
{
  public:
    PropPlanner() : Node("prop_planner")
    {
        auto const topic = declare_parameter<std::string>("odom_topic", "/odometry/filtered/global");

        // SensorDataQoS accepts both best-effort and reliable odometry publishers.
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            topic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_cb(msg); });

        // OccupancyGrid is the planner's temporary map interface. If the mapping
        // system later publishes another data structure (for example, obstacle
        // markers), add an adapter that converts it to this grid representation
        // rather than coupling the A* search to the mapping message format.
        auto const map_topic = declare_parameter<std::string>("global_map_topic", "/prop_planner/mock_map");
        global_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic, rclcpp::QoS(1).reliable().transient_local(),
            [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { global_map_cb(msg); });

        plan_service_ = create_service<nav_msgs::srv::GetPlan>(
            "~/plan", [this](nav_msgs::srv::GetPlan::Request::SharedPtr request,
                            nav_msgs::srv::GetPlan::Response::SharedPtr response)
            { response->plan = plan(*request); });

        RCLCPP_INFO(get_logger(), "Waiting for odometry on %s", odom_sub_->get_topic_name());
    }

  private:
    // A synchronous, four-connected A* search. The map must already include
    // vessel clearance. Unknown cells are blocked; no TF or smoothing is implied.
    // Keep this algorithm operating on a grid. A future non-OccupancyGrid map
    // should be translated by an adapter before it reaches this function.
    nav_msgs::msg::Path plan(nav_msgs::srv::GetPlan::Request const &request)
    {
        auto fail = [this](char const *reason)
        {
            RCLCPP_WARN(get_logger(), "Planning failed: %s", reason);
            return nav_msgs::msg::Path{};
        };
        if (!last_global_map_) return fail("no map received");
        auto const &map = *last_global_map_;
        auto const &info = map.info;
        auto const &origin = info.origin;
        auto const count = static_cast<uint64_t>(info.width) * info.height;
        if (!info.width || !info.height || count > 1000000 || count != map.data.size() ||
            !std::isfinite(info.resolution) || info.resolution <= 0 ||
            !std::isfinite(origin.position.x) || !std::isfinite(origin.position.y) ||
            map.header.frame_id.empty())
            return fail("invalid map metadata or map exceeds one million cells");
        // Explicitly reject rotated grids instead of silently misplacing obstacles.
        auto const &q = origin.orientation;
        if (!(std::abs(q.x) < 1e-6 && std::abs(q.y) < 1e-6 &&
              std::abs(q.z) < 1e-6 && std::abs(std::abs(q.w) - 1.0) < 1e-6))
            return fail("only axis-aligned maps are supported");
        if (request.tolerance != 0.0)
            return fail("only exact goals (tolerance = 0) are supported");
        auto start = request.start;
        if (start.header.frame_id.empty())
        {
            if (!has_odom_) return fail("no odometry received for implicit start");
            start.header = last_odom_.header;
            start.pose = last_odom_.pose.pose;
        }
        if (start.header.frame_id != map.header.frame_id ||
            request.goal.header.frame_id != map.header.frame_id)
            return fail("start, goal, and map must share a frame; no TF conversion is performed");
        auto cell = [&](geometry_msgs::msg::PoseStamped const &pose) -> int
        {
            auto const x = std::floor((pose.pose.position.x - origin.position.x) / info.resolution);
            auto const y = std::floor((pose.pose.position.y - origin.position.y) / info.resolution);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(pose.pose.position.z) ||
                x < 0 || y < 0 || x >= info.width || y >= info.height) return -1;
            return static_cast<int>(y) * static_cast<int>(info.width) + static_cast<int>(x);
        };
        auto free = [&](int index) { return map.data[index] >= 0 && map.data[index] < 50; };
        int const first = cell(start), goal = cell(request.goal);
        if (first < 0 || goal < 0) return fail("start or goal outside map or non-finite");
        if (!free(first) || !free(goal)) return fail("start or goal occupied or unknown");
        int const width = static_cast<int>(info.width), height = static_cast<int>(info.height);
        auto heuristic = [&](int index)
        { return std::abs(index % width - goal % width) + std::abs(index / width - goal / width); };
        std::vector<int> cost(count, std::numeric_limits<int>::max()), parent(count, -1);
        using Entry = std::pair<int, int>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
        cost[first] = 0;
        open.emplace(heuristic(first), first);
        while (!open.empty())
        {
            auto const [score, current] = open.top();
            open.pop();
            if (score != cost[current] + heuristic(current)) continue;
            if (current == goal) break;
            int const x = current % width, y = current / width;
            int const neighbors[] = {x > 0 ? current - 1 : -1, x + 1 < width ? current + 1 : -1,
                                     y > 0 ? current - width : -1, y + 1 < height ? current + width : -1};
            for (int next : neighbors)
            {
                if (next < 0 || !free(next) || cost[next] <= cost[current] + 1) continue;
                cost[next] = cost[current] + 1;
                parent[next] = current;
                open.emplace(cost[next] + heuristic(next), next);
            }
        }
        if (cost[goal] == std::numeric_limits<int>::max()) return fail("no route exists");
        std::vector<int> route;
        for (int index = goal; index != -1; index = parent[index]) route.push_back(index);
        std::reverse(route.begin(), route.end());
        nav_msgs::msg::Path path;
        path.header.frame_id = map.header.frame_id;
        path.header.stamp = now();
        start.header = path.header;
        path.poses.push_back(start);
        // Retain endpoint cell centers so the endpoint connectors stay inside free cells.
        for (int index : route)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = origin.position.x + (index % width + 0.5) * info.resolution;
            pose.pose.position.y = origin.position.y + (index / width + 0.5) * info.resolution;
            pose.pose.position.z = start.pose.position.z;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        auto end = request.goal;
        end.header = path.header;
        path.poses.push_back(end);
        return path;
    }

    void global_map_cb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr const &msg)
    {
        last_global_map_ = msg;
    }

    void odom_cb(nav_msgs::msg::Odometry::ConstSharedPtr const &msg)
    {
        last_odom_ = *msg;
        has_odom_ = true;

        auto const &position = last_odom_.pose.pose.position;
        // Use a steady clock so terminal logging is also throttled when simulation time pauses.
        RCLCPP_INFO_THROTTLE(get_logger(), log_clock_, 1000,
                             "Boat position in frame '%s': x=%.2f m, y=%.2f m, z=%.2f m",
                             last_odom_.header.frame_id.c_str(), position.x, position.y, position.z);
    }

    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr plan_service_;
    rclcpp::Clock log_clock_{ RCL_STEADY_TIME };
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_sub_;
    // Null until the first map arrives; retain the message without copying the grid.
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr last_global_map_;
    nav_msgs::msg::Odometry last_odom_;
    // Implicit-start requests check this flag before using last_odom_.
    // Receiving (0, 0, 0) is valid; it is different from receiving no message.
    bool has_odom_{ false };
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PropPlanner>());
    rclcpp::shutdown();
    return 0;
}
