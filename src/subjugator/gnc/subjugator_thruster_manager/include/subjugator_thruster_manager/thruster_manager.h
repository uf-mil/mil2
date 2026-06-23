#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "subjugator_msgs/msg/thruster_efforts.hpp"
class ThrusterManager : public rclcpp::Node
{
  public:
    ThrusterManager();

  private:
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    Eigen::VectorXd reference_wrench_;  // TODO: change to Vector6d
    Eigen::MatrixXd tam_;
    Eigen::VectorXd current_velocities_;  // 6-element vector: [u, v, w, p, q, r] (linear and angular velocities)
    Eigen::Quaterniond current_orientation_;
    Eigen::MatrixXd M_;  // Mass matrix (6x6) diagonal
    bool heard_odom_;
    int const dof_ = 6;
    int const thruster_count_ = 8;
    double thruster_cap_;
    double max_force_pos_;
    double max_force_neg_;

    // Gravity/buoyancy compensation parameters and derived constants
    double vehicle_mass_kg_;
    double vehicle_volume_m3_;
    double water_density_;
    double x_cog_to_cob_m_;
    double y_cog_to_cob_m_;
    double z_cog_to_cob_m_;
    double W_;  // weight force (N)
    double B_;  // buoyancy force (N)

    // Damping coefficients (linear and quadratic), 6-element diagonals
    Eigen::VectorXd D_lin_;
    Eigen::VectorXd D_quad_;

    // Working vectors reused in the timer callback
    Eigen::Vector3d linear_velocities;
    Eigen::Vector3d angular_velocities;
    Eigen::VectorXd damping_linear;
    Eigen::VectorXd damping_quadratic;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<subjugator_msgs::msg::ThrusterEfforts>::SharedPtr thrust_publisher_;

    Eigen::Matrix3d skew_symmetric(Eigen::Vector3d const &v);
    double get_roll(Eigen::Quaterniond const &q);
    double get_pitch(Eigen::Quaterniond const &q);

    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
    void wrench_callback(geometry_msgs::msg::Wrench::SharedPtr msg);
    void timer_callback();
};
