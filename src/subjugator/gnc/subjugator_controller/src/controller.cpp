#include "subjugator_controller/controller.h"

PIDController::PIDController() : Node("pid_controller")
{
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("subjugator_localization/odometry/filtered", 10,
                                                                   [this](nav_msgs::msg::Odometry::UniquePtr msg)
                                                                   { this->odom_cb(std::move(msg)); });
    sub_goal_trajectory_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "trajectory", 10,
        [this](geometry_msgs::msg::Pose::UniquePtr msg) { this->goal_trajectory_cb(std::move(msg)); });
    pub_cmd_wrench_ = this->create_publisher<geometry_msgs::msg::Wrench>("cmd_wrench", 1);

    // init gains as ros parameters for dynamic reconfiguration
    this->declare_parameter("kp", std::vector<double>(dof_, 0.0));
    this->declare_parameter("ki", std::vector<double>(dof_, 0.0));
    this->declare_parameter("kd", std::vector<double>(dof_, 0.0));
    this->declare_parameter("imax", std::vector<double>(dof_, 0.0));
    this->declare_parameter("imin", std::vector<double>(dof_, 0.0));
    this->declare_parameter("antiwindup", std::vector<double>(dof_, 0.0));  // maybe should be bool?

    std::vector<double> kp_arr_ = this->get_parameter("kp").as_double_array();
    std::vector<double> ki_arr_ = this->get_parameter("ki").as_double_array();
    std::vector<double> kd_arr_ = this->get_parameter("kd").as_double_array();
    std::vector<double> imax_arr_ = this->get_parameter("imax").as_double_array();
    std::vector<double> imin_arr_ = this->get_parameter("imin").as_double_array();
    std::vector<double> antiwindup_arr_ = this->get_parameter("antiwindup").as_double_array();

    // TODO: log starting gains to screen

    for (size_t i = 0; i < pid_vec_.size(); i++)
    {
        pid_vec_[i] =
            control_toolbox::Pid(kp_arr_[i], ki_arr_[i], kd_arr_[i], imax_arr_[i], imin_arr_[i], antiwindup_arr_[i]);
    }

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    auto param_cb = [this](rclcpp::Parameter const &p)
    {
        RCLCPP_INFO(this->get_logger(), "cb: Received an update to parameter \"%s\"", p.get_name().c_str());

        std::vector<double> kp_arr_ = this->get_parameter("kp").as_double_array();
        std::vector<double> ki_arr_ = this->get_parameter("ki").as_double_array();
        std::vector<double> kd_arr_ = this->get_parameter("kd").as_double_array();
        std::vector<double> imax_arr_ = this->get_parameter("imax").as_double_array();
        std::vector<double> imin_arr_ = this->get_parameter("imin").as_double_array();
        std::vector<double> antiwindup_arr_ = this->get_parameter("antiwindup").as_double_array();

        for (size_t i = 0; i < pid_vec_.size(); i++)
        {
            pid_vec_[i].set_gains(kp_arr_[i], ki_arr_[i], kd_arr_[i], imax_arr_[i], imin_arr_[i], antiwindup_arr_[i]);
        }
    };

    kp_cb_handle_ = this->param_subscriber_->add_parameter_callback("kp", param_cb);
    ki_cb_handle_ = this->param_subscriber_->add_parameter_callback("ki", param_cb);
    kd_cb_handle_ = this->param_subscriber_->add_parameter_callback("kd", param_cb);
    imax_cb_handle_ = this->param_subscriber_->add_parameter_callback("imax", param_cb);
    imin_cb_handle_ = this->param_subscriber_->add_parameter_callback("imin", param_cb);
    antiwindup_cb_handle_ = this->param_subscriber_->add_parameter_callback("antiwindup", param_cb);

    last_cmd_time_ = this->get_clock()->now();

    control_loop();
}

void PIDController::control_loop()
{
    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        RCLCPP_INFO(this->get_logger(), "control loop");

        rclcpp::Time tnow = this->get_clock()->now();
        double dt_s = (tnow - last_cmd_time_).seconds();

        // init error and command arrays
        Eigen::Matrix<double, 6, 1> errors = Eigen::Matrix<double, 6, 1>::Zero();
        std::array<double, 6> commands = { 0 };

        // compute position error
        errors(Eigen::seq(0, 2)) = last_goal_trajectory_(Eigen::seq(0, 2)) - last_odom_(Eigen::seq(0, 2));
        RCLCPP_INFO(this->get_logger(), "error: '%s'", std::to_string(errors[0]).c_str());

        // TODO: compute orientation error

        // errors -> commands
        for (size_t i = 0; i < pid_vec_.size(); i++)
        {
            commands[i] = pid_vec_[i].compute_command(errors[i], dt_s);
        }

        // publish as cmd_wrench
        publish_commands(commands);
        RCLCPP_INFO(this->get_logger(), "command: '%s'", std::to_string(commands[0]).c_str());

        last_cmd_time_ = tnow;
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }
}

void PIDController::publish_commands(std::array<double, 6> const &commands)
{
    auto msg = geometry_msgs::msg::Wrench();
    msg.force.x = commands[0];
    msg.force.y = commands[1];
    msg.force.z = commands[2];
    msg.torque.x = commands[3];
    msg.torque.y = commands[4];
    msg.torque.z = commands[5];
    pub_cmd_wrench_->publish(msg);
}

void PIDController::odom_cb(nav_msgs::msg::Odometry::UniquePtr const msg)
{
    // last_odom_ = *msg;
    last_odom_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w;
    RCLCPP_INFO(this->get_logger(), "heard odom: '%s'", std::to_string(msg->pose.pose.position.x).c_str());
}

void PIDController::goal_trajectory_cb(geometry_msgs::msg::Pose::UniquePtr const msg)
{
    // last_goal_trajectory_ = *msg;
    last_goal_trajectory_ << msg->position.x, msg->position.y, msg->position.z, msg->orientation.x, msg->orientation.y,
        msg->orientation.z, msg->orientation.w;
    RCLCPP_INFO(this->get_logger(), "heard goal: '%s'", std::to_string(msg->position.x).c_str());
}
