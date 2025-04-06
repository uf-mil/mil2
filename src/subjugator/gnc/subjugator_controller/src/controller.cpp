#include "subjugator_controller/controller.h"

PIDController::PIDController() : Node("pid_controller")
{
    this->is_shutdown = false;

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10, [this](nav_msgs::msg::Odometry::UniquePtr msg) { this->odom_cb(std::move(msg)); });
    sub_goal_trajectory_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "trajectory", 10,
        [this](geometry_msgs::msg::Pose::UniquePtr msg) { this->goal_trajectory_cb(std::move(msg)); });
    pub_cmd_wrench_ = this->create_publisher<geometry_msgs::msg::Wrench>("cmd_wrench", rclcpp::QoS(1).reliable());

    // callback to send 0 cmd_wrench on shutdown
    using rclcpp::contexts::get_global_default_context;
    get_global_default_context()->add_pre_shutdown_callback(
        [this]()
        {
            this->is_shutdown = true;
            this->shutdown();
        });

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    auto param_cb = [this](rclcpp::Parameter const &p)
    {
        RCLCPP_INFO(this->get_logger(), "cb: Received an update to parameter \"%s\"", p.get_name().c_str());

        param_map_[p.get_name()].first = p.as_double_array();

        for (size_t i = 0; i < pid_vec_.size(); i++)
        {
            pid_vec_[i].set_gains(param_map_["kp"].first[i], param_map_["ki"].first[i], param_map_["kd"].first[i],
                                  param_map_["imax"].first[i], param_map_["imin"].first[i],
                                  param_map_["antiwindup"].first[i]);
        }
    };

    std::vector<std::string> const params = { "kp", "ki", "kd", "imax", "imin", "antiwindup" };
    for (auto const &param : params)
    {
        this->declare_parameter(param, std::vector<double>(dof_, 0.0));
        param_map_[param].first = this->get_parameter(param).as_double_array();
        param_map_[param].second = this->param_subscriber_->add_parameter_callback(param, param_cb);
    }

    // TODO: log starting gains to screen

    for (size_t i = 0; i < pid_vec_.size(); i++)
    {
        pid_vec_[i] = control_toolbox::Pid(param_map_["kp"].first[i], param_map_["ki"].first[i],
                                           param_map_["kd"].first[i], param_map_["imax"].first[i],
                                           param_map_["imin"].first[i], param_map_["antiwindup"].first[i]);
    }

    // set starting command to all zeros
    last_cmd_time_ = this->get_clock()->now();
    last_goal_trajectory_ = Eigen::Matrix<double, 7, 1>::Zero();
    last_goal_trajectory_[6] = 1.0;  // set w to 1.0 for quaternion

    // init odom to all zeros
    last_odom_ = Eigen::Matrix<double, 7, 1>::Zero();
    last_odom_[6] = 1.0;  // set w to 1.0 for quaternion

    control_loop();
}

void PIDController::control_loop()
{
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !this->is_shutdown)
    {
        RCLCPP_INFO(this->get_logger(), "control loop");

        rclcpp::Time tnow = this->get_clock()->now();
        double dt_s = (tnow - last_cmd_time_).seconds();

        // init error and command arrays
        Eigen::Matrix<double, 6, 1> errors = Eigen::Matrix<double, 6, 1>::Zero();
        std::array<double, 6> commands = { 0 };

        // compute position error
        errors(Eigen::seq(0, 2)) = last_goal_trajectory_(Eigen::seq(0, 2)) - last_odom_(Eigen::seq(0, 2));

        // compute orientation error
        Eigen::Quaterniond goal_quat = Eigen::Quaterniond(last_goal_trajectory_(6), last_goal_trajectory_(3),
                                                          last_goal_trajectory_(4), last_goal_trajectory_(5));
        Eigen::Quaterniond odom_quat = Eigen::Quaterniond(last_odom_(6), last_odom_(3), last_odom_(4), last_odom_(5));

        // perform calculation on quaternions, then convert to euler angles
        // source:
        // https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-attitudetran#mjx-eqn%3Aeq%3Aq2dcm
        Eigen::Matrix3d dcm = goal_quat.toRotationMatrix() * odom_quat.inverse().toRotationMatrix();
        double roll = atan2(dcm(2, 1), dcm(2, 2));
        double pitch = atan2(-dcm(2, 0), sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));
        double yaw = atan2(dcm(1, 0), dcm(0, 0));
        errors(Eigen::seq(3, 5)) = Eigen::Vector3d(roll, pitch, yaw);

        // apply PID control to errors
        for (size_t i = 0; i < pid_vec_.size(); i++)
        {
            commands[i] = pid_vec_[i].compute_command(errors[i], dt_s);
        }

        // publish as cmd_wrench
        publish_commands(commands);
        RCLCPP_INFO(this->get_logger(), "errors: '%s' '%s' '%s' '%s' '%s' '%s'", std::to_string(errors[0]).c_str(),
                    std::to_string(errors[1]).c_str(), std::to_string(errors[2]).c_str(),
                    std::to_string(errors[3]).c_str(), std::to_string(errors[4]).c_str(),
                    std::to_string(errors[5]).c_str());
        RCLCPP_INFO(this->get_logger(), "commands: '%s' '%s' '%s' '%s' '%s' '%s'", std::to_string(commands[0]).c_str(),
                    std::to_string(commands[1]).c_str(), std::to_string(commands[2]).c_str(),
                    std::to_string(commands[3]).c_str(), std::to_string(commands[4]).c_str(),
                    std::to_string(commands[5]).c_str());

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
    last_odom_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w;
}

void PIDController::goal_trajectory_cb(geometry_msgs::msg::Pose::UniquePtr const msg)
{
    last_goal_trajectory_ << msg->position.x, msg->position.y, msg->position.z, msg->orientation.x, msg->orientation.y,
        msg->orientation.z, msg->orientation.w;
    RCLCPP_INFO(this->get_logger(), "heard goal: '%s'", std::to_string(msg->position.x).c_str());
}

void PIDController::shutdown()
{
    std::array<double, 6> commands = { 0 };
    publish_commands(commands);
    pub_cmd_wrench_->wait_for_all_acked();
    RCLCPP_INFO(this->get_logger(), "Shutting down PID controller, sent 0 cmd_wrench.");
}
