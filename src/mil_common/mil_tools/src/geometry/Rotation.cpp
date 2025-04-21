#include "mil_tools/geometry/Rotation.hpp"

namespace mil_tools::geometry
{

Rotation::Rotation(std::array<double, 3> const& rot_vec)
{
    double cy = cos(rot_vec[2] * 0.5);
    double sy = sin(rot_vec[2] * 0.5);
    double cp = cos(rot_vec[1] * 0.5);
    double sp = sin(rot_vec[1] * 0.5);
    double cr = cos(rot_vec[0] * 0.5);
    double sr = sin(rot_vec[0] * 0.5);
    quat_.w() = cy * cp * cr + sy * sp * sr;
    quat_.x() = cy * cp * sr - sy * sp * cr;
    quat_.y() = sy * cp * sr + cy * sp * cr;
    quat_.z() = sy * cp * cr - cy * sp * sr;
}

Rotation::Rotation(std::array<double, 4> const& quat)
{
    quat_ = Eigen::Quaterniond(quat[3], quat[1], quat[2], quat[0]);
}

Rotation::Rotation(Eigen::Quaterniond const& quat)
{
    quat_ = quat;
}

Rotation::Rotation(Eigen::Vector3d const& rot_vec)
{
    double cy = cos(rot_vec[2] * 0.5);
    double sy = sin(rot_vec[2] * 0.5);
    double cp = cos(rot_vec[1] * 0.5);
    double sp = sin(rot_vec[1] * 0.5);
    double cr = cos(rot_vec[0] * 0.5);
    double sr = sin(rot_vec[0] * 0.5);
    quat_.w() = cy * cp * cr + sy * sp * sr;
    quat_.x() = cy * cp * sr - sy * sp * cr;
    quat_.y() = sy * cp * sr + cy * sp * cr;
    quat_.z() = sy * cp * cr - cy * sp * sr;
}

Rotation::Rotation(Eigen::Vector4d const& quat)
{
    quat_ = Eigen::Quaterniond(quat[3], quat[0], quat[1], quat[2]);
}

Rotation::Rotation(geometry_msgs::msg::Quaternion const& quat)
{
    quat_ = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}

Rotation::Rotation(geometry_msgs::msg::Vector3 const& rot_vec)
{
    double cy = cos(rot_vec.z * 0.5);
    double sy = sin(rot_vec.z * 0.5);
    double cp = cos(rot_vec.y * 0.5);
    double sp = sin(rot_vec.y * 0.5);
    double cr = cos(rot_vec.x * 0.5);
    double sr = sin(rot_vec.x * 0.5);
    quat_.w() = cy * cp * cr + sy * sp * sr;
    quat_.x() = cy * cp * sr - sy * sp * cr;
    quat_.y() = sy * cp * sr + cy * sp * cr;
    quat_.z() = sy * cp * cr - cy * sp * sr;
}

Rotation::Rotation(std::initializer_list<double> const& init_list)
{
    if (init_list.size() == 3)
    {
        auto it = init_list.begin();
        double cy = cos(*(it + 2) * 0.5);
        double sy = sin(*(it + 2) * 0.5);
        double cp = cos(*(it + 1) * 0.5);
        double sp = sin(*(it + 1) * 0.5);
        double cr = cos(*it * 0.5);
        double sr = sin(*it * 0.5);
        quat_.w() = cy * cp * cr + sy * sp * sr;
        quat_.x() = cy * cp * sr - sy * sp * cr;
        quat_.y() = sy * cp * sr + cy * sp * cr;
        quat_.z() = sy * cp * cr - cy * sp * sr;
    }
    else if (init_list.size() == 4)
    {
        auto it = init_list.begin();
        quat_ = Eigen::Quaterniond(*(it + 3), *(it), *(it + 1), *(it + 2));
    }
    else
    {
        throw std::invalid_argument("Invalid initializer list size for Rotation constructor");
    }
}

Rotation::Rotation()
{
    quat_ = Eigen::Quaterniond(1, 0, 0, 0);
}

Eigen::Vector3d Rotation::rot_vec(Axis const& first, Axis const& second, Axis const& third) const
{
    return quat_.toRotationMatrix().eulerAngles(static_cast<int>(first), static_cast<int>(second),
                                                static_cast<int>(third));
}

std::tuple<double, double, double> Rotation::rot_vec_tuple(Axis const& first, Axis const& second,
                                                           Axis const& third) const
{
    Eigen::Vector3d rot_vec_eigen{ rot_vec(first, second, third) };
    return { rot_vec_eigen[0], rot_vec_eigen[1], rot_vec_eigen[2] };
}

Eigen::Matrix3d Rotation::rot_mat() const
{
    return quat_.toRotationMatrix();
}

geometry_msgs::msg::Quaternion Rotation::quat_msg() const
{
    geometry_msgs::msg::Quaternion quat;
    quat.w = quat_.w();
    quat.x = quat_.x();
    quat.y = quat_.y();
    quat.z = quat_.z();
    return quat;
}

geometry_msgs::msg::Vector3 Rotation::rot_vec_msg() const
{
    geometry_msgs::msg::Vector3 res;
    Eigen::Vector3d rot_vec_eigen{ rot_vec() };
    res.x = rot_vec_eigen[0];
    res.y = rot_vec_eigen[1];
    res.z = rot_vec_eigen[2];
    return res;
}

Eigen::Vector4d Rotation::quat_vec() const
{
    Eigen::Vector4d quat_vec;
    quat_vec << quat_.w(), quat_.x(), quat_.y(), quat_.z();
    return quat_vec;
}

bool Rotation::operator==(Rotation const& other) const
{
    return quat_.w() == other.quat_w() && quat_.x() == other.quat_x() && quat_.y() == other.quat_y() &&
           quat_.z() == other.quat_z();
}

bool Rotation::operator!=(Rotation const& other) const
{
    return !(*this == other);
}

Rotation Rotation::operator+(double scalar) const
{
    return Rotation{ quat_.x(), quat_.y(), quat_.z(), quat_.w() + scalar };
}

Rotation Rotation::operator*(Rotation const& other) const
{
    return Rotation(quat_ * other.quat_);
}

Rotation Rotation::operator/(double scalar) const
{
    return Rotation(Eigen::Vector4d{ quat_.x() / scalar, quat_.y() / scalar, quat_.z() / scalar, quat_.w() / scalar });
}

Rotation Rotation::operator*(double scalar) const
{
    return Rotation(Eigen::Vector4d{ quat_.x() * scalar, quat_.y() * scalar, quat_.z() * scalar, quat_.w() * scalar });
}

Rotation Rotation::operator-() const
{
    // operator-() not defined on Eigen::Quaternion
    return Rotation(Eigen::Vector4d{ -quat_.x(), -quat_.y(), -quat_.z(), quat_.w() });
}

void Rotation::normalize()
{
    quat_.normalize();
}

void Rotation::inverse()
{
    quat_ = quat_.inverse();
}

void Rotation::conjugate()
{
    quat_ = quat_.conjugate();
}

std::basic_ostream<char>& operator<<(std::basic_ostream<char>& os, Rotation const& obj)
{
    os << "Rotation<x=" << obj.quat_x() << " y=" << obj.quat_y() << " z=" << obj.quat_z() << " w=" << obj.quat_w()
       << " roll_deg=" << obj.roll_deg() << " pitch_deg=" << obj.pitch_deg() << " yaw_deg=" << obj.yaw_deg() << ">";
    return os;
}

Rotation Rotation::imaginary() const
{
    return Rotation(Eigen::Vector4d{ quat_.x(), quat_.y(), quat_.z(), 0 });
}

}  // namespace mil_tools::geometry
