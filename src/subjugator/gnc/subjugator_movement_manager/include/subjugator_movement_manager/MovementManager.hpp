#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <subjugator_msgs/msg/absolute_move.hpp>
#include <subjugator_msgs/msg/move_status.hpp>
#include <subjugator_msgs/msg/relative_move.hpp>

class MovementManager : public rclcpp::Node
{
  public:
    MovementManager();

  private:
    // Frame-tagged internal form that both input messages resolve into.
    struct MoveRequest
    {
        bool relative{ false };
        geometry_msgs::msg::Pose target;  // absolute pose, or body-frame offset when relative
        uint8_t hold{ 0 };                // AbsoluteMove HOLD_* bitmask; ignored when relative
        bool precise{ false };            // tighter tolerances + longer timeout
        uint32_t command_id{ 0 };
    };

    void absolute_move_cb(subjugator_msgs::msg::AbsoluteMove::SharedPtr const &msg);
    void relative_move_cb(subjugator_msgs::msg::RelativeMove::SharedPtr const &msg);
    void odom_cb(nav_msgs::msg::Odometry::SharedPtr const &msg);

    // Timer step: run the active move's arrival state machine and publish status.
    void publish_status_();

    void drive_to_(MoveRequest const &req);

    // Turn a request into an absolute goal in the odom frame.
    geometry_msgs::msg::Pose resolve_goal_(MoveRequest const &req) const;

    geometry_msgs::msg::Pose base_pose_(MoveRequest const &req) const;

    bool current_errors_(geometry_msgs::msg::Pose const &goal, double &dist, double &ang) const;

    // Publish one MoveStatus for the active command in the given state.
    void emit_status_(uint8_t state, std::string const &message, double dist, double ang);

    void reject_no_odom_(uint32_t command_id);

    rclcpp::Subscription<subjugator_msgs::msg::AbsoluteMove>::SharedPtr absolute_move_sub_;
    rclcpp::Subscription<subjugator_msgs::msg::RelativeMove>::SharedPtr relative_move_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_pub_;
    rclcpp::Publisher<subjugator_msgs::msg::MoveStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::optional<nav_msgs::msg::Odometry> latest_odom_;
    std::optional<geometry_msgs::msg::Pose> last_goal_;

    bool have_active_command_{ false };
    uint32_t active_command_id_{ 0 };
    geometry_msgs::msg::Pose active_goal_;
    uint8_t active_state_{ 0 };
    bool active_precise_{ false };
    rclcpp::Time active_start_;

    double pos_tol{ 0.2 };
    double ori_tol{ 12.0 };
    double loose_pos_tol{ 0.85 };
    double loose_ori_tol{ 40.0 };
    double precise_pos_tol{ 0.08 };
    double precise_ori_tol{ 4.0 };
    double grace_secs{ 5.0 };

    double active_tight_pos_{ 0.2 };
    double active_tight_ori_{ 12.0 };
    double active_loose_pos_{ 0.85 };
    double active_loose_ori_{ 40.0 };
    double active_timeout_s_{ 0.0 };
};
