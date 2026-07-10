#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

struct ADRC {
    Eigen::Matrix3d AESO;
    Eigen::Vector3d bESO;

    double b0;
    double k1;
    double l1, l2;

    double u_min, u_max;
    double x1, x2;
};

class ControllerNode : public rclcpp::Node {
    std::vector<double> b0s{}, omegaCLs{}, kESOs{};
    double Tsample = 0.1;
    std::array<ADRC, 6> controllers;

    rclcpp::ParameterEventHandler param_subscriber;
    std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> param_cbs;

    std::unordered_map<std::string, std::vector<double>*> param_lut;

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
    }

    void update_params() {
        for (int i = 0; i < 6; ++i) {
            ADRC &c = controllers[i];

            // locations in z-domain for pole placement
            double zCL = exp(-omegaCLs[i] * Tsample);
            double zESO = exp(-kESOs[i] * omegaCLs[i] * Tsample);

            // controller and observer gains
            c.k1 = (1.0 - zCL) / Tsample;
            c.b0 = b0s[i];
            c.l1 = 1.0 - zESO * zESO;
            c.l2 = (1.0 - zESO) * (1.0 - zESO) / Tsample;

            // observer matrix
            c.AESO(0, 0) = 1.0 - c.l1;
            c.AESO(0, 1) = Tsample * (1.0 - c.l1);
            c.AESO(1, 0) = -c.l2;
            c.AESO(1, 1) = 1.0 - Tsample * c.l2;
            c.bESO(0) = c.b0 * Tsample * (1.0 - c.l1);
            c.bESO(1) = -c.b0 * Tsample * c.l2;

            controllers[i] = c;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
