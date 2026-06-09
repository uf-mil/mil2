#include "common.hpp"

nav_msgs::msg::Odometry extrapolate_odom(nav_msgs::msg::Odometry &odom) {
    return odom;
}

State subtract_odom(nav_msgs::msg::Odometry &odom_prev,
                    nav_msgs::msg::Odometry &odom_next) {
    State s{};
    auto &pp = odom_prev.pose, &pn = odom_next.pose;

    // phi: yaw_next - yaw_prev
    auto &qp = pp.pose.orientation, &qn = pn.pose.orientation;
    tf2::Quaternion q_prev{qp.x, qp.y, qp.z, qp.w};
    tf2::Quaternion q_next{qn.x, qn.y, qn.z, qn.w};

    tf2::Matrix3x3 mat_prev{q_prev}, mat_next{q_next};
    double r, p, yaw_prev, yaw_next;
    mat_prev.getRPY(r, p, yaw_prev);
    mat_next.getRPY(r, p, yaw_next);
    s.phi = yaw_next - yaw_prev;

    // position: R1^-1 (t2 - t1)
    double dx = pn.pose.position.x - pp.pose.position.x;
    double dy = pn.pose.position.y - pp.pose.position.y;
    s.x = dx * cos(yaw_prev)  + dy * sin(yaw_prev);
    s.y = dx * -sin(yaw_prev) + dy * cos(yaw_prev);
    s.cov[0] = 0.1; //>0< 1  2
    s.cov[4] = 0.1; // 3 >4< 5
    s.cov[8] = 0.1; // 6  7 >8<

    // std::cout << "  " << dx * 100 << ", " << dy * 100 << std::endl;
    // std::cout << s.x * 100 << ", " << s.y * 100 << ", " << s.phi << std::endl;

    return s;
}
