#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <sys/types.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "subjugator_msgs/msg/thruster_efforts.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <subjugator_msgs/srv/servo.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

struct Context
{
    rclcpp::Node::SharedPtr node;

    // Publishers and Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_pub;
    rclcpp::Publisher<subjugator_msgs::msg::ThrusterEfforts>::SharedPtr raw_effort_pub;  // no mutex since im lazy and
                                                                                         // no-one else using this rn :P
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detections_sub;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr tracking_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wall_direction_sub;

    // Down-cam perception (Task 5). Separate streams so both YOLO nodes
    // (front + down) can run concurrently; consumers pick a camera below.
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr down_targets_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr down_image_sub;

    // Servo service clients (matched to services exposed by servo_controller/driver.py
    // on the real robot, or the GripperControl plugin in sim). Used by ActuateServo.
    rclcpp::Client<subjugator_msgs::srv::Servo>::SharedPtr dropper_client;
    rclcpp::Client<subjugator_msgs::srv::Servo>::SharedPtr gripper_client;
    rclcpp::Client<subjugator_msgs::srv::Servo>::SharedPtr torpedo_client;
    // Service clients to actuate servos (driver.py services)
    // services and clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr controller_enable_client;

    // Latest state
    std::mutex odom_mx;
    std::optional<nav_msgs::msg::Odometry> latest_odom;

    std::mutex last_goal_mx;
    std::optional<geometry_msgs::msg::Pose> last_goal;

    // Raw per-frame YOLO detections (/yolo/detections).
    std::mutex detections_mx;
    std::optional<yolo_msgs::msg::DetectionArray> latest_detections;

    // Tracker output (/yolo/tracking): smoothed boxes with track ids. The tracker
    // copies the full detection through, so this is the stream that carries the
    // board's corner keypoints needed for arching (see tracking_node.py).
    std::mutex tracking_mx;
    std::optional<yolo_msgs::msg::DetectionArray> latest_tracking;
    // Latest class from the coin_flip classifier node (/coin_flip/direction).
    std::mutex wall_direction_mx;
    std::optional<std::string> latest_wall_direction;

    // PID of the coin_flip_node the mission launched (-1 = none), so it can be
    // killed on shutdown.
    std::mutex child_mx;
    pid_t coin_flip_pid{ -1 };

    std::mutex img_mx;
    uint32_t img_width{ 0 };
    uint32_t img_height{ 0 };

    std::mutex down_detections_mx;
    std::optional<yolo_msgs::msg::DetectionArray> latest_down_detections;

    std::mutex down_img_mx;
    uint32_t down_img_width{ 0 };
    uint32_t down_img_height{ 0 };

    // Robot role (Task 5). Cross-task state: written by Task 1 (gate side) at
    // runtime via set_role; read by SelectTarget. Seeded by a startup param
    // until Task 1 wiring exists. "" = unknown.
    std::mutex role_mx;
    std::string role;

    std::string get_role()
    {
        std::scoped_lock lk(role_mx);
        return role;
    }
    void set_role(std::string const& r)
    {
        std::scoped_lock lk(role_mx);
        role = r;
    }

    // Named absolute waypoints captured during a mission (or preloaded).
    // Mission BT nodes (RememberWaypoint / LookupWaypoint) read and write this.
    std::mutex waypoints_mx;
    std::unordered_map<std::string, geometry_msgs::msg::Pose> waypoints;

    inline rclcpp::Logger logger() const
    {
        return node->get_logger();
    }

    // Publish an orientation/position goal and cache it as last_goal in one
    // step, so the "publish, then remember what we asked for" invariant can
    // never drift apart. Every motion node that emits a goal (CenterCamera,
    // HoneBearing, SearchForTarget, DescendUntilDetected, AlignDepth, AlignYaw,
    // YawStyle) previously repeated this publish + scoped_lock pair by hand.
    inline void command_goal(geometry_msgs::msg::Pose const& goal)
    {
        goal_pub->publish(goal);
        std::scoped_lock lk(last_goal_mx);
        last_goal = goal;
    }

    // Latest detections for the requested camera ("down" selects the down-cam
    // stream; anything else falls back to the front-cam stream). Returns a copy
    // so callers don't hold the mutex while iterating.
    inline std::optional<yolo_msgs::msg::DetectionArray> detections_for(std::string const& camera)
    {
        if (camera == "down")
        {
            std::scoped_lock lk(down_detections_mx);
            return latest_down_detections;
        }
        std::scoped_lock lk(detections_mx);
        return latest_detections;
    }

    // Image size for the requested camera. Fills w/h and returns true once a
    // frame has been seen (both nonzero); false on cold start.
    inline bool image_size_for(std::string const& camera, uint32_t& w, uint32_t& h)
    {
        if (camera == "down")
        {
            std::scoped_lock lk(down_img_mx);
            w = down_img_width;
            h = down_img_height;
        }
        else
        {
            std::scoped_lock lk(img_mx);
            w = img_width;
            h = img_height;
        }
        return w != 0 && h != 0;
    }
};

// Fetch the shared Context from the "ctx" blackboard port into `ctx` if it is
// not already set, logging a uniform error on failure. Returns true when `ctx`
// is usable. Collapses the guard every BT node repeated verbatim at the top of
// onStart/onRunning/tick:
//   if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_)) { RCLCPP_ERROR(...); return FAILURE; }
// Templated on the node type so this header needs no BehaviorTree dependency --
// node.getInput is only instantiated at the (BT-aware) call site.
template <class Node>
inline bool require_ctx(Node& node, std::shared_ptr<Context>& ctx, char const* who)
{
    if (ctx)
    {
        return true;
    }
    if (!node.template getInput<std::shared_ptr<Context>>("ctx", ctx) || !ctx)
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "%s: missing ctx", who);
        return false;
    }
    return true;
}

#define REGISTER(name)                                                                                                 \
    extern BT::BehaviorTreeFactory factory;                                                                            \
    extern "C" __attribute__((constructor)) void register##name()                                                      \
    {                                                                                                                  \
        factory.registerNodeType<name>(#name);                                                                         \
    }
