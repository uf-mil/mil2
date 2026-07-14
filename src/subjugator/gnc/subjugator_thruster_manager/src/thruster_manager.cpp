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

// Construct node class
ThrusterManager::ThrusterManager() : Node("thruster_manager")
{
    reference_wrench_ = Eigen::VectorXd::Zero(6);
    this->declare_parameter("thruster_cap", 0.0);
    thruster_cap_ = this->get_parameter("thruster_cap").as_double();

    // The force<->effort curve (lut.h) already encodes each direction's max
    // thrust, so the cap is given in effort and converted once to the force
    // limits it implies (forward and reverse, which differ).
    cap_force_pos_ = force_from_effort(thruster_cap_);
    cap_force_neg_ = force_from_effort(-thruster_cap_);

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
    // Per-thruster force (Newtons) that realizes the requested wrench.
    Eigen::VectorXd forces(tam_.completeOrthogonalDecomposition().pseudoInverse() * reference_wrench_);

    // Saturation: if any thruster needs more force than the cap permits, scale
    // every force down by a single factor so the wrench direction is preserved.
    // We saturate in force units (not effort) because the force->effort curve is
    // nonlinear, so scaling efforts would distort the relative thrust mix. The
    // forward and reverse limits differ, so each thruster is compared to the one
    // matching its direction.
    double worst_ratio = 1.0;
    for (int i = 0; i < forces.size(); i++)
    {
        double const limit = (forces[i] >= 0) ? cap_force_pos_ : cap_force_neg_;
        if (limit != 0.0)
        {
            worst_ratio = std::max(worst_ratio, forces[i] / limit);
        }
    }
    if (worst_ratio > 1.0)
    {
        forces /= worst_ratio;
    }

    // Map each force to a normalized effort through the asymmetric, nonlinear LUT
    // (replaces the old single-slope linear scaling) for thruster board compatibility.
    Eigen::VectorXd thrust_values(forces.size());
    for (int i = 0; i < forces.size(); i++)
    {
        thrust_values[i] = effort_from_force(forces[i]);
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
