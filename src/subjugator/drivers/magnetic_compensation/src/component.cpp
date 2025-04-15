#include <memory>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace mil::magnetic_compensation
{

class HardsoftCompensator : public rclcpp::Node
{
  public:
    HardsoftCompensator(rclcpp::NodeOptions const &options) : Node("hardsoft_compensator", options)
    {
        // Declare and get parameters
        frame_id_ = this->declare_parameter<std::string>("frame_id", "imu_link");

        auto scale = this->declare_parameter<std::vector<double>>("scale");
        auto shift_vec = this->declare_parameter<std::vector<double>>("shift");

        if (scale.size() != 9 || shift_vec.size() != 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid parameter sizes for scale or shift");
            throw std::runtime_error("Invalid parameter sizes");
        }

        // Convert to Eigen
        Eigen::Matrix3d scale_matrix;
        for (size_t i = 0; i < 9; ++i)
        {
            scale_matrix(i / 3, i % 3) = scale[i];
        }

        // Check if the scale matrix is invertible
        if (scale_matrix.determinant() == 0)
        {
            std::string scale_str;
            for (size_t i = 0; i < 9; ++i)
            {
                scale_str += std::to_string(scale[i]) + " ";
            }
            RCLCPP_ERROR(this->get_logger(), "Scale matrix is not invertible: %s", scale_str.c_str());
            throw std::runtime_error("Scale matrix is not invertible");
        }
        scale_inverse_ = scale_matrix.inverse();
        shift_ = Eigen::Vector3d(shift_vec[0], shift_vec[1], shift_vec[2]);

        // Create publisher and subscriber
        pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
        sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
            "/imu/mag_raw", 10,
            [this](sensor_msgs::msg::MagneticField::SharedPtr const msg) { this->handle_mag_field(msg); });
    }

  private:
    std::string frame_id_;
    Eigen::Matrix3d scale_inverse_;
    Eigen::Vector3d shift_;

    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_;

    void handle_mag_field(sensor_msgs::msg::MagneticField::SharedPtr const msg)
    {
        if (msg->header.frame_id != frame_id_)
        {
            RCLCPP_ERROR(this->get_logger(), "msg's frame_id != configured frame_id! Ignoring message.");
            return;
        }

        Eigen::Vector3d raw;
        tf2::fromMsg(msg->magnetic_field, raw);
        Eigen::Vector3d processed = scale_inverse_ * (raw - shift_);

        sensor_msgs::msg::MagneticField result;
        result.header = msg->header;
        result.header.stamp = this->now();
        tf2::toMsg(processed, result.magnetic_field);

        pub_->publish(result);
    }
};

}  // namespace mil::magnetic_compensation

RCLCPP_COMPONENTS_REGISTER_NODE(mil::magnetic_compensation::HardsoftCompensator)
