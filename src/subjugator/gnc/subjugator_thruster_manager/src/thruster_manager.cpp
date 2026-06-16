#include "subjugator_thruster_manager/thruster_manager.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

#include "geometry_msgs/msg/wrench.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "subjugator_msgs/msg/thruster_efforts.hpp"

// ============================================================================
// PHYSICAL PARAMETERS — MEASURE THESE FROM YOUR ACTUAL AUV
// ============================================================================
//
// HOW TO MEASURE:
//
//   VEHICLE_MASS_KG:
//     Weigh the fully-assembled sub (with all electronics, batteries, etc.)
//     on a scale in air. Convert lbs to kg if needed (1 lb = 0.4536 kg).
//
//   VEHICLE_VOLUME_M3:
//     Measure or calculate the total displaced water volume of the hull.
//     For a cylinder: V = pi * r^2 * length
//     Or submerge the sub in a known container and measure water displaced.
//     Convert liters to m^3 if needed (1 liter = 0.001 m^3).
//
//   Z_COG_TO_COB_M:
//     This is the vertical distance (in meters) from the Center of Gravity
//     to the Center of Buoyancy, measured along the sub's body Z-axis.
//     Positive means CoB is ABOVE CoG (this is the stable configuration).
//
//     To estimate empirically:
//       1. Command a known pitch angle (e.g. 10 degrees)
//       2. Read the steady-state torque the PID outputs to hold that angle
//       3. z_G = steady_state_torque / (VEHICLE_MASS_KG * 9.81 * sin(pitch_rad))
//
//     Start with 0.0 and tune upward in small increments (e.g. 0.01 m steps)
//     until roll is stable. Typical values for a small AUV: 0.02 to 0.10 m.
//
// ============================================================================

// Construct node class
ThrusterManager::ThrusterManager() : Node("thruster_manager")
{
    reference_wrench_ = Eigen::VectorXd::Zero(6);
    heard_odom_ = false;
    current_orientation_ = Eigen::Quaterniond::Identity();

    // -----------------------------------------------------------------------
    // Existing thruster parameters (unchanged)
    // -----------------------------------------------------------------------
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
                                         "with ros2 launch, and that config values are set.");
    }

    // -----------------------------------------------------------------------
    // Gravity/buoyancy compensation parameters
    // Set these in your launch/config file, or edit the defaults below.
    // -----------------------------------------------------------------------

    // MEASURE: Total mass of the sub in kilograms (weigh it fully assembled)
    this->declare_parameter("vehicle_mass_kg", 0.0);
    // ^^^ DEFAULT 0.0 — replace with your measured mass, e.g. 13.5

    // MEASURE: Total displaced water volume in cubic meters
    this->declare_parameter("vehicle_volume_m3", 0.0);
    // ^^^ DEFAULT 0.0 — replace with your calculated/measured hull volume, e.g. 0.0138

    // MEASURE/TUNE: Vertical distance from CoG to CoB in meters.
    // Positive = CoB is above CoG (stable). Start at 0.0, tune upward.
    this->declare_parameter("z_cog_to_cob_m", 0.0);
    // ^^^ DEFAULT 0.0 — start here and tune empirically (see notes above)

    // Water density — 1000.0 for fresh water, 1025.0 for salt water
    // ADJUST: Change to 1025.0 if competing/testing in salt water
    this->declare_parameter("water_density_kg_m3", 1000.0);

    vehicle_mass_kg_ = this->get_parameter("vehicle_mass_kg").as_double();
    vehicle_volume_m3_ = this->get_parameter("vehicle_volume_m3").as_double();
    z_cog_to_cob_m_ = this->get_parameter("z_cog_to_cob_m").as_double();
    water_density_ = this->get_parameter("water_density_kg_m3").as_double();

    // Derived constants (computed once at startup)
    // W = weight force in Newtons (downward)
    // B = buoyancy force in Newtons (upward)
    W_ = vehicle_mass_kg_ * 9.81;
    B_ = water_density_ * vehicle_volume_m3_ * 9.81;

    RCLCPP_INFO(this->get_logger(), "Gravity compensation: W=%.2f N, B=%.2f N, z_G=%.4f m, net=%.2f N", W_, B_,
                z_cog_to_cob_m_, W_ - B_);

    if (std::abs(vehicle_mass_kg_) < 1e-6)
    {
        RCLCPP_WARN(this->get_logger(), "vehicle_mass_kg is 0.0 — gravity compensation is disabled. "
                                        "Set this parameter in your launch/config file.");
    }

    // -----------------------------------------------------------------------
    // Subscriptions and publishers
    // -----------------------------------------------------------------------

    wrench_subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "cmd_wrench", 1,
        [this](geometry_msgs::msg::Wrench::SharedPtr const msg) -> void { this->wrench_callback(msg); });

    // Subscribe to odometry so we know the current orientation for compensation
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 1,
        [this](nav_msgs::msg::Odometry::SharedPtr const msg) -> void { this->odom_callback(msg); });

    thrust_publisher_ = this->create_publisher<subjugator_msgs::msg::ThrusterEfforts>("thruster_efforts", 1);
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", thrust_publisher_->get_topic_name());

    timer_ = this->create_wall_timer(std::chrono::milliseconds(250), [this]() -> void { this->timer_callback(); });
}

// ---------------------------------------------------------------------------
// Store the latest odometry orientation
// ---------------------------------------------------------------------------
void ThrusterManager::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_orientation_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                              msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    heard_odom_ = true;
}

// ---------------------------------------------------------------------------
// Update reference wrench when new cmd_wrench is received
// ---------------------------------------------------------------------------
void ThrusterManager::wrench_callback(geometry_msgs::msg::Wrench::SharedPtr msg)
{
    this->reference_wrench_ << msg->force.x, msg->force.y, msg->force.z, msg->torque.x, msg->torque.y, msg->torque.z;
}

// ---------------------------------------------------------------------------
// Helper: extract roll (phi) from quaternion using rotation matrix
// Roll is rotation about body X axis.
// ---------------------------------------------------------------------------
double ThrusterManager::get_roll(Eigen::Quaterniond const &q)
{
    Eigen::Matrix3d R = q.toRotationMatrix();
    return std::atan2(R(2, 1), R(2, 2));
}

// ---------------------------------------------------------------------------
// Helper: extract pitch (theta) from quaternion using rotation matrix
// Pitch is rotation about body Y axis.
// ---------------------------------------------------------------------------
double ThrusterManager::get_pitch(Eigen::Quaterniond const &q)
{
    Eigen::Matrix3d R = q.toRotationMatrix();
    return std::atan2(-R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
}

// ---------------------------------------------------------------------------
// Compute and publish thruster efforts, with gravity/buoyancy compensation
// ---------------------------------------------------------------------------
void ThrusterManager::timer_callback()
{
    Eigen::VectorXd compensated_wrench = reference_wrench_;

    // Apply gravity/buoyancy restoring force compensation (Fossen g(eta) vector)
    // Only apply if we have a valid orientation reading and mass is configured
    if (heard_odom_ && std::abs(vehicle_mass_kg_) > 1e-6)
    {
        double phi = get_roll(current_orientation_);     // roll angle (rad)
        double theta = get_pitch(current_orientation_);  // pitch angle (rad)

        // --- Restoring FORCES (Fossen g(eta), rows 0-2) ---
        // These are the net gravity-buoyancy force components in the body frame.
        // Near-neutrally-buoyant subs have W ≈ B so these are small, but included
        // for correctness — especially if your sub is slightly heavy or light.
        double fx = (W_ - B_) * std::sin(theta);
        double fy = -(W_ - B_) * std::cos(theta) * std::sin(phi);
        double fz = -(W_ - B_) * std::cos(theta) * std::cos(phi);

        // --- Restoring TORQUES (Fossen g(eta), rows 3-5) ---
        // These are the dominant terms for pitch/roll stability.
        // They are proportional to z_cog_to_cob_m_ — if CoG and CoB are perfectly
        // aligned (z=0), these vanish entirely and no torque compensation is needed.
        //
        // NOTE: If you have physically aligned CoG and CoB, set z_cog_to_cob_m=0.0
        // and only the force rows above will contribute. You can tune z_cog_to_cob_m
        // to add artificial restoring torque if the physical alignment isn't perfect.
        double tx = z_cog_to_cob_m_ * W_ * std::cos(theta) * std::sin(phi);  // roll restoring
        double ty = z_cog_to_cob_m_ * W_ * std::sin(theta);                  // pitch restoring
        double tz = 0.0;  // yaw has no gravity/buoyancy restoring torque (by definition)

        // Subtract restoring forces from the commanded wrench so thrusters
        // counteract gravity/buoyancy effects on top of the PID command
        compensated_wrench(0) -= fx;
        compensated_wrench(1) -= fy;
        compensated_wrench(2) -= fz;
        compensated_wrench(3) -= tx;
        compensated_wrench(4) -= ty;
        // compensated_wrench(5) — yaw left unchanged

        // Uncomment to debug compensation values:
        /*
        RCLCPP_INFO(this->get_logger(),
                    "Compensation: phi=%.3f theta=%.3f | F=[%.2f %.2f %.2f] T=[%.2f %.2f]",
                    phi, theta, fx, fy, fz, tx, ty);
        */
    }
    else if (!heard_odom_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "No odometry received yet — gravity compensation inactive.");
    }

    // Compute thruster efforts via pseudoinverse of TAM
    Eigen::VectorXd thrust_values(tam_.completeOrthogonalDecomposition().pseudoInverse() * compensated_wrench);

    // Scale thrust values and check against thruster cap (unchanged from original)
    double biggest_thrust = 0;
    bool over_thruster_cap = false;
    for (int i = 0; i < thrust_values.size(); i++)
    {
        // Scale force (N) to percent effort for thruster board compatibility.
        // NOTE: max_force_pos_ and max_force_neg_ must be set in your config.
        // MEASURE: These are the max thrust your thrusters produce in each
        //          direction (forward vs reverse). Check your thruster datasheet
        //          (e.g. Blue Robotics T200: ~5.1 kgf forward, ~3.7 kgf reverse)
        //          and convert to Newtons (1 kgf = 9.81 N).
        thrust_values[i] =
            (thrust_values[i] > 0) ? thrust_values[i] / max_force_pos_ : thrust_values[i] / max_force_neg_;

        if (std::abs(thrust_values[i]) > biggest_thrust)
        {
            biggest_thrust = std::abs(thrust_values[i]);
            if (biggest_thrust > thruster_cap_)
            {
                over_thruster_cap = true;
            }
        }
    }

    // Rescale all thrusters proportionally if any exceeds the cap
    if (over_thruster_cap)
    {
        thrust_values = thrust_values * (thruster_cap_ / biggest_thrust);
    }

    // Publish thruster efforts
    // NOTE: The thruster index mapping below (frh, flh, brh, blh, frv, flv, brv, blv)
    // must match your physical thruster numbering and TAM column order.
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
