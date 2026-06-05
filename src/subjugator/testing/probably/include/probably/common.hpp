// precompiled headers
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "mrpt/slam/CRangeBearingKFSLAM2D.h"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "yolo_msgs/msg/detection_array.hpp"

struct State {
    double x, y, phi;
    double cov[9];
};

nav_msgs::msg::Odometry extrapolate_odom(nav_msgs::msg::Odometry &odom);

State subtract_odom(nav_msgs::msg::Odometry &odom_prev,
                    nav_msgs::msg::Odometry &odom_next);
