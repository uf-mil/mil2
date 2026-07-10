#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

struct ADRC {
    Eigen::Matrix2d AESO;
    Eigen::Vector2d bESO;

    double b0;
    Eigen::RowVector2d K;
    Eigen::Vector2d L;

    double u_min, u_max;
    Eigen::Vector2d X;

    double compute(double e) {
        Eigen::Vector2d F = X + L * e;

        double u = (K * F)(0) / b0;
        double u_lim = std::clamp(u, u_min, u_max);

        // prepare internal state variables for next time step
        X = AESO * F - bESO * u_lim;
        return u_lim;
    }
};

class ControllerNode : public rclcpp::Node {
    std::vector<double> b0s{}, omegaCLs{}, kESOs{};
    double Tsample = 0.1;
    std::array<ADRC, 6> controllers;

    rclcpp::ParameterEventHandler param_subscriber;
    std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> param_cbs;

    std::unordered_map<std::string, std::vector<double>*> param_lut;

    geometry_msgs::msg::Pose goal;
    bool has_goal = false;

    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_sub;

public:
    ControllerNode() : Node("adrc_controller"), param_subscriber(this) {
        param_lut = {
            {"b0", &b0s},
            {"omegaCL", &omegaCLs},
            {"kESO", &kESOs}
        };

        auto param_cb = [this](const rclcpp::Parameter &p) {
            RCLCPP_INFO(get_logger(), "Received update to parameter \"%s\"",
                        p.get_name().c_str());

            std::string name = p.get_name();
            if (name == "Tsample") {
                Tsample = get_parameter(name).as_double();
            } else {
                *param_lut[name] = get_parameter(name).as_double_array();
            }

            update_params();
        };

        declare_parameter("Tsample", Tsample);
        Tsample = get_parameter("Tsample").as_double();
        param_cbs.emplace_back(param_subscriber.add_parameter_callback("Tsample", param_cb));

        for (auto [name, param] : param_lut) {
            declare_parameter(name, std::vector<double>(6, 0.0));
            *param = get_parameter(name).as_double_array();
            param_cbs.emplace_back(param_subscriber.add_parameter_callback(name, param_cb));
        }

        update_params();

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10, [this](nav_msgs::msg::Odometry odom) { odom_cb(odom); });
    }

    void update_params() {
        for (int i = 0; i < 6; ++i) {
            ADRC &c = controllers[i];

            // locations in z-domain for pole placement
            double zCL = exp(-omegaCLs[i] * Tsample);
            double zESO = exp(-kESOs[i] * omegaCLs[i] * Tsample);

            // controller and observer gains
            c.K(0) = (1.0 - zCL) / Tsample;
            c.K(1) = 1;
            c.b0 = b0s[i];
            double l1 = 1.0 - zESO * zESO;
            double l2 = (1.0 - zESO) * (1.0 - zESO) / Tsample;
            c.L(0) = l1;
            c.L(1) = l2;

            // observer matrix
            c.AESO(0, 0) = 1.0 - l1;
            c.AESO(0, 1) = Tsample * (1.0 - l1);
            c.AESO(1, 0) = -l2;
            c.AESO(1, 1) = 1.0 - Tsample * l2;
            c.bESO(0) = c.b0 * Tsample * (1.0 - l1);
            c.bESO(1) = -c.b0 * Tsample * l2;

            controllers[i] = c;
        }
    }

    void odom_cb(nav_msgs::msg::Odometry &odom) {
        geometry_msgs::msg::Point gp = goal.position, op = odom.pose.pose.position;
        geometry_msgs::msg::Quaternion gq = goal.orientation, oq = odom.pose.pose.orientation;

        if (!has_goal) {
            goal = odom.pose.pose;
            has_goal = true;
            return;
        }

        // compute orientation error
        Eigen::Quaterniond goal_quat = Eigen::Quaterniond(gq.w, gq.x, gq.y, gq.z);
        Eigen::Quaterniond odom_quat = Eigen::Quaterniond(oq.w, oq.x, oq.y, oq.z);

        Eigen::Vector3d ypr = (goal_quat * odom_quat.inverse()).canonicalEulerAngles(2, 1, 0);

        std::array<double, 6> errors = {
            gp.x - op.x,
            gp.y - op.y,
            gp.z - op.z,
            ypr(2),
            ypr(1),
            ypr(0),
        };

        // apply PID control to errors
        for (int i = 0; i < 6; ++i) {
            controllers[i].compute(errors[i]);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
