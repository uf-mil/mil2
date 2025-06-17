#include "subjugator_thruster_manager/thruster_manager.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/string.hpp"
#include "subjugator_msgs/msg/thruster_efforts.hpp"

// Construct node class
ThrusterManager::ThrusterManager() : Node("thruster_manager")
{
    reference_wrench_ = Eigen::VectorXd::Zero(6);
    this->declare_parameter("thruster_cap", 0.0);
    thruster_cap_ = this->get_parameter("thruster_cap").as_double();

    this->declare_parameter("max_force_pos", 0.0);
    max_force_pos_ = this->get_parameter("max_force_pos").as_double();
    this->declare_parameter("max_force_neg", 0.0);
    max_force_neg_ = this->get_parameter("max_force_neg").as_double();

    // Create thruster allocation matrix from config file parameters
    tam_ = Eigen::MatrixXd::Zero(dof_, thruster_count_);
    std::vector<std::string> axes = { "x", "y", "z", "rx", "ry", "rz" };
    for (unsigned int i = 0; i < axes.size(); i++)
    {
        std::string param_name = "tam." + axes[i];
        this->declare_parameter(param_name, std::vector<double>(thruster_count_, 0.0));
        tam_.row(i) = Eigen::Map<Eigen::RowVectorXd const>(this->get_parameter(param_name).as_double_array().data(),
                                                           thruster_count_);
    }
    // Throw error if TAM is all zeroes (ran without params instead of launched)
    if (tam_.isZero())
    {
        RCLCPP_ERROR(this->get_logger(), "Thruster Allocation Matrix is all zero. Ensure you are launching the node "
                                         "with "
                                         "ros2 launch, and that config values are set.");
    }

    wrench_subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "cmd_wrench", 1,
        [this](geometry_msgs::msg::Wrench::SharedPtr const msg) -> void { this->wrench_callback(msg); });

    thrust_publisher_ = this->create_publisher<subjugator_msgs::msg::ThrusterEfforts>("thruster_efforts", 1);
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", thrust_publisher_->get_topic_name());
    timer_ = this->create_wall_timer(std::chrono::milliseconds(250), [this]() -> void { this->timer_callback(); });
}

// Update wrench when new msg heard
void ThrusterManager::wrench_callback(geometry_msgs::msg::Wrench::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Heard wrench: [%.2f, %.2f, %.2f], [%.2f, %.2f, %.2f]", msg->force.x,
    // msg->force.y,
    //             msg->force.z, msg->torque.x, msg->torque.y, msg->torque.z);
    this->reference_wrench_ << msg->force.x, msg->force.y, msg->force.z, msg->torque.x, msg->torque.y, msg->torque.z;
}

// Compute and publish thruster efforts
void ThrusterManager::timer_callback()
{
    Eigen::VectorXd thrust_values(tam_.completeOrthogonalDecomposition().pseudoInverse() * reference_wrench_);

    // check that the allocated thrust is not over the thruster cap (typically 1.0), and if it is, rescale all thrusters
    double biggest_thrust = 0;
    bool over_thruster_cap = false;
    for (int i = 0; i < thrust_values.size(); i++)
    {
        // scale desired force to percent effort for thruster board compatibility
        // Note that thrusters produce different force if spinning forward (max_force_pos_) vs spinning backwards
        thrust_values[i] =
            (thrust_values[i] > 0) ? thrust_values[i] / max_force_pos_ : thrust_values[i] / max_force_neg_;
        // determine largest thrust magnitude
        if (std::abs(thrust_values[i]) > biggest_thrust)
        {
            biggest_thrust = std::abs(thrust_values[i]);
            if (biggest_thrust > thruster_cap_)
            {
                over_thruster_cap = true;
            }
        }
    }
    if (over_thruster_cap)
    {
        thrust_values = thrust_values * (thruster_cap_ / biggest_thrust);
    }

    auto msg = subjugator_msgs::msg::ThrusterEfforts();
    msg.thrust_frh = thrust_values[0];
    msg.thrust_flh = thrust_values[1];
    msg.thrust_brh = thrust_values[2];
    msg.thrust_blh = thrust_values[3];
    msg.thrust_frv = thrust_values[4];
    msg.thrust_flv = thrust_values[5];
    msg.thrust_brv = thrust_values[6];
    msg.thrust_blv = thrust_values[7];

    this->thrust_publisher_->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThrusterManager>());
    rclcpp::shutdown();

    return 0;
}
