#pragma once

#include <array>
#include <cmath>
#include <initializer_list>
#include <ostream>

#include <Eigen/Dense>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mil::geometry
{

enum class Axis
{
    X = 0,
    Y = 1,
    Z = 2
};

class Rotation
{
    Eigen::Quaterniond quat_;

  public:
    ////////////////////////////////////////
    // Helpful constructors
    ////////////////////////////////////////
    // Also equivalent to Rotation()
    static Rotation identity()
    {
        return { Eigen::Quaterniond(1, 0, 0, 0) };
    }

    ////////////////////////////////////////
    // Constructors
    ////////////////////////////////////////
    Rotation(std::array<double, 3> const& rot_vec);
    // x, y, z, w
    Rotation(std::array<double, 4> const& quat);
    Rotation(Eigen::Quaterniond const& quat);
    Rotation(Eigen::Vector3d const& rot_vec);
    // x, y, z, w
    Rotation(Eigen::Vector4d const& quat);
    Rotation(geometry_msgs::msg::Quaternion const& quat);
    Rotation(geometry_msgs::msg::Vector3 const& rot_vec);
    Rotation(std::initializer_list<double> const& init_list);
    Rotation();

    ////////////////////////////////////////
    // Quaternion accessors
    ////////////////////////////////////////
    [[nodiscard]] inline Eigen::Quaterniond quat() const
    {
        return quat_;
    }
    [[nodiscard]] inline double quat_w() const
    {
        return quat_.w();
    };
    // Alias for quat_w()
    [[nodiscard]] inline double real() const
    {
        return quat_.w();
    }
    [[nodiscard]] inline double quat_x() const
    {
        return quat_.x();
    };
    [[nodiscard]] inline double quat_y() const
    {
        return quat_.y();
    };
    [[nodiscard]] inline double quat_z() const
    {
        return quat_.z();
    };
    [[nodiscard]] inline double norm() const
    {
        return quat_.norm();
    };
    [[nodiscard]] inline double abs_imag() const
    {
        return std::sqrt(quat_.x() * quat_.x() + quat_.y() * quat_.y() + quat_.z() * quat_.z());
    }

    ////////////////////////////////////////
    // Euler accessors
    ////////////////////////////////////////
    [[nodiscard]] inline double roll() const
    {
        return rot_vec(Axis::X, Axis::Y, Axis::Z)[0];
    };
    [[nodiscard]] inline double pitch() const
    {
        return rot_vec(Axis::X, Axis::Y, Axis::Z)[1];
    };
    [[nodiscard]] inline double yaw() const
    {
        return rot_vec(Axis::X, Axis::Y, Axis::Z)[2];
    };
    [[nodiscard]] inline double roll_deg() const
    {
        return roll() * 180 / M_PI;
    };
    [[nodiscard]] inline double pitch_deg() const
    {
        return pitch() * 180 / M_PI;
    };
    [[nodiscard]] inline double yaw_deg() const
    {
        return yaw() * 180 / M_PI;
    };

    [[nodiscard]] Eigen::Vector3d rot_vec(Axis const& first = Axis::X, Axis const& second = Axis::Y,
                                          Axis const& third = Axis::Z) const;
    [[nodiscard]] std::tuple<double, double, double> rot_vec_tuple(Axis const& first = Axis::X,
                                                                   Axis const& second = Axis::Y,
                                                                   Axis const& third = Axis::Z) const;
    [[nodiscard]] Eigen::Matrix3d rot_mat() const;
    [[nodiscard]] geometry_msgs::msg::Quaternion quat_msg() const;
    [[nodiscard]] geometry_msgs::msg::Vector3 rot_vec_msg() const;
    [[nodiscard]] Eigen::Vector4d quat_vec() const;

    ////////////////////////////////////////
    // Operators
    ////////////////////////////////////////
    bool operator==(Rotation const& other) const;
    bool operator!=(Rotation const& other) const;
    Rotation operator+(double scalar) const;
    Rotation operator*(Rotation const& other) const;
    Rotation operator*(double scalar) const;
    Rotation operator/(double scalar) const;
    Rotation operator-() const;
    friend std::basic_ostream<char>& operator<<(std::basic_ostream<char>& os, Rotation const& obj);

    ////////////////////////////////////////
    // Math
    ////////////////////////////////////////
    // void pow(double exponent);
    // void pow(Rotation const& exponent);

    ////////////////////////////////////////
    // Special operations
    ////////////////////////////////////////
    void normalize();
    void inverse();
    void conjugate();
    [[nodiscard]] Rotation imaginary() const;
};

}  // namespace mil::geometry
