#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "../include/navigator_path_planner/lqrrt_planner.hpp"
#include "../include/navigator_path_planner/action/move.hpp"

class PathPlannerNode : public rclcpp::Node {
public:
  using Move = path_planner_cpp::action::Move;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

  PathPlannerNode() : Node("path_planner_node"), planner_() {
    RCLCPP_INFO(get_logger(), "Path Planner Node started.");
    action_server_ = rclcpp_action::create_server<Move>(
      this,
      "move_to",
      std::bind(&PathPlannerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PathPlannerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&PathPlannerNode::handle_accepted, this, std::placeholders::_1)
    );
  }

private:
  LqrrtPlanner planner_;
  rclcpp_action::Server<Move>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const Move::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Received goal request (%.2f, %.2f)",
                goal->target_pose.pose.position.x,
                goal->target_pose.pose.position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(get_logger(), "Goal canceled");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    std::thread([this, goal_handle]() {
      auto result = std::make_shared<Move::Result>();
      Eigen::VectorXd start(3), goal(3);
      start << 0.0, 0.0, 0.0;
      goal << goal_handle->get_goal()->target_pose.pose.position.x,
              goal_handle->get_goal()->target_pose.pose.position.y,
              0.0;
      auto path = planner_.plan(start, goal);
      result->success = !path.empty();
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal finished.");
    }).detach();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
