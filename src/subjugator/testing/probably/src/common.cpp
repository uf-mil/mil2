#include "common.hpp"

nav_msgs::msg::Odometry extrapolate_odom(nav_msgs::msg::Odometry &odom) {
    return odom;
}

State subtract_odom(nav_msgs::msg::Odometry &odom_prev,
                    nav_msgs::msg::Odometry &odom_next) {
    State s{};
    auto &pp = odom_prev.pose, &pn = odom_next.pose;
    s.x = pn.pose.position.x - pp.pose.position.x;
    s.y = pn.pose.position.y - pp.pose.position.y;

    auto &qp = pp.pose.orientation, &qn = pn.pose.orientation;
    tf2::Quaternion q_prev{qp.x, qp.y, qp.z, qp.w};
    tf2::Quaternion q_next{qn.x, qn.y, qn.z, qn.w};

    tf2::Matrix3x3 mat{q_next - q_prev};
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    s.phi = yaw;

    return s;
}
