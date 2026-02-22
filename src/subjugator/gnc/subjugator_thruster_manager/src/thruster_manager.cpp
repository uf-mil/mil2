#include "subjugator_thruster_manager/thruster_manager.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/string.hpp"
#include "subjugator_msgs/msg/thruster_efforts.hpp"
#include "subjugator_thruster_manager/lut.h"

// chat made this but i passed dsa so it's ok
int binarySearchClosest(std::array<double, 201> const &arr, double target)
{
    size_t n = arr.size();
    if (n == 0)
        return -1;

    int left = 0, right = n - 1;

    // Handle out-of-range cases early
    if (target <= arr[0])
        return 0;
    if (target >= arr[n - 1])
        return n - 1;

    while (left <= right)
    {
        int mid = left + (right - left) / 2;

        if (arr[mid] == target)
            return mid;

        if (arr[mid] < target)
            left = mid + 1;
        else
            right = mid - 1;
    }

    // Now right < left
    // right is the largest element < target
    // left is the smallest element > target

    if (abs(arr[left] - target) < abs(arr[right] - target))
        return left;
    else
        return right;
}

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
    // TODO only need to compute this once
    Eigen::VectorXd thrust_values(tam_.completeOrthogonalDecomposition().pseudoInverse() * reference_wrench_);

    RCLCPP_WARN(this->get_logger(), "---------------");
    RCLCPP_WARN(this->get_logger(), "frh: %.3f", thrust_values[0]);
    RCLCPP_WARN(this->get_logger(), "flh: %.3f", thrust_values[1]);
    RCLCPP_WARN(this->get_logger(), "brh: %.3f", thrust_values[2]);
    RCLCPP_WARN(this->get_logger(), "blh: %.3f", thrust_values[3]);
    RCLCPP_WARN(this->get_logger(), "frv: %.3f", thrust_values[4]);
    RCLCPP_WARN(this->get_logger(), "flv: %.3f", thrust_values[5]);
    RCLCPP_WARN(this->get_logger(), "brv: %.3f", thrust_values[6]);
    RCLCPP_WARN(this->get_logger(), "blv: %.3f", thrust_values[7]);
    RCLCPP_WARN(this->get_logger(), "---------------");

    // find thrust with max magnitude
    double biggest_thrust = 0;
    for (int i = 0; i < thrust_values.size(); i++)
    {
        auto thrust_mag = std::abs(thrust_values[i]);
        biggest_thrust = std::max(biggest_thrust, thrust_mag);
    }

    // if that biggest_thrust is too big, scale the entire wrench down
    // i.e keep the intention, but reduce the magnitude
    bool over_thruster_cap = biggest_thrust > max_force_neg_;  // cap all the time
    if (over_thruster_cap)                                     // scale?
    {
        thrust_values = thrust_values * (max_force_neg_ / biggest_thrust);
    }

    // binary search each thrust (units of force in newtons) to find its index in the lut
    // we can then do:
    // lut index -> desired pwm duty
    // and finally desired pwm duty -> percentage [-1,1]
    for (int i = 0; i < thrust_values.size(); i++)
    {
        if (std::abs(thrust_values[i]) < 1e-9)
        {
            thrust_values[i] = 0.0;
            continue;
        }

        int index = binarySearchClosest(thrust_lut, thrust_values[i]);
        double duty = 1100.0 + double(index) * 4.0;  // index will be [0:200] so result will be 1100 to 1900

        // what is the map function doing on the pico?
        // see here:
        // ((x--100)*(1900-1100)/(100--100)+1100)
        // duty = 400(x+1) + 1100 (where x=[-1:1]
        //
        // so we just solve for x (we have duty
        thrust_values[i] = (duty - 1100.0) / 400.0 - 1.0;
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
