#include "subjugator_trajectory_planner/trajectory_planner.h"

TrajectoryPlanner::TrajectoryPlanner() : Node("trajectory_planner")
{
    this->heard_newer_path_ = false;
    this->heard_odom_ = false;

    // make goal tolerance a param
    this->declare_parameter("goal_tolerance", 0.5);
    this->goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10, [this](nav_msgs::msg::Odometry::UniquePtr msg) { this->odom_cb(std::move(msg)); });
    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        "path", 10, [this](nav_msgs::msg::Path::UniquePtr msg) { this->path_cb(std::move(msg)); });
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("goal/trajectory", 1);

    RCLCPP_INFO(this->get_logger(), "Started trajectory planner / breadcrumb manager. Waiting to hear path and odom to "
                                    "start.");

    rclcpp::Rate rate(10);
    while (!heard_newer_path_ || !heard_odom_)
    {
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }

    handle_paths();
}

void TrajectoryPlanner::handle_paths()
{
    while (rclcpp::ok())
    {
        if (heard_newer_path_)
        {
            RCLCPP_INFO(this->get_logger(), "Handling new path.");
            heard_newer_path_ = false;

            // iterate to find which pose to send as the first goal
            size_t first_goal_index = 0;
            double last_dist = std::numeric_limits<double>::max(), curr_dist = 0.0;
            for (size_t i = 0; i < path_->poses.size(); i++)
            {
                curr_dist = nav2_util::geometry_utils::euclidean_distance(this->odom_->pose.pose,
                                                                          this->path_->poses[i].pose, true);
                if (curr_dist > last_dist)
                {
                    first_goal_index = i;
                    break;
                }
                last_dist = curr_dist;
                first_goal_index = i;
            }

            // publish each goal in order, waiting to reach each one
            for (size_t i = first_goal_index; i < path_->poses.size(); i++)
            {
                // publish goal
                auto goal_msg = geometry_msgs::msg::Pose();
                goal_msg.position.x = path_->poses[i].pose.position.x;
                goal_msg.position.y = path_->poses[i].pose.position.y;
                goal_msg.position.z = path_->poses[i].pose.position.z;
                goal_msg.orientation.x = path_->poses[i].pose.orientation.x;
                goal_msg.orientation.y = path_->poses[i].pose.orientation.y;
                goal_msg.orientation.z = path_->poses[i].pose.orientation.z;
                goal_msg.orientation.w = path_->poses[i].pose.orientation.w;
                goal_publisher_->publish(goal_msg);
                RCLCPP_INFO(this->get_logger(), "Publishing goal: %f, %f, %f, %f, %f, %f, %f", goal_msg.position.x,
                            goal_msg.position.y, goal_msg.position.z, goal_msg.orientation.x, goal_msg.orientation.y,
                            goal_msg.orientation.z, goal_msg.orientation.w);

                // spin and check for odom to be in range of goal to move on
                double dist_to_goal =
                    nav2_util::geometry_utils::euclidean_distance(this->odom_->pose.pose, goal_msg, true);
                this->goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
                while (dist_to_goal > this->goal_tolerance_ && !heard_newer_path_)
                {
                    rclcpp::spin_some(this->get_node_base_interface());
                    dist_to_goal =
                        nav2_util::geometry_utils::euclidean_distance(this->odom_->pose.pose, goal_msg, true);
                }
                if (heard_newer_path_)
                {
                    break;
                }
            }
        }
        else
        {
            // wait for a new path
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }
}

void TrajectoryPlanner::odom_cb(nav_msgs::msg::Odometry::UniquePtr msg)
{
    this->odom_ = std::move(msg);
    this->heard_odom_ = true;
}

void TrajectoryPlanner::path_cb(nav_msgs::msg::Path::UniquePtr msg)
{
    this->path_ = std::move(msg);
    this->heard_newer_path_ = true;
}
