#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mil_tools::geometry
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
    Rotation(std::array<double, 3> const& rot_vec)
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
    // x, y, z, w
    Rotation(std::array<double, 4> const& quat)
    {
        quat_ = Eigen::Quaterniond(quat[3], quat[1], quat[2], quat[0]);
    }
    Rotation(Eigen::Quaterniond const& quat)
    {
        quat_ = quat;
    }
    Rotation(Eigen::Vector3d const& rot_vec)
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
    // x, y, z, w
    Rotation(Eigen::Vector4d const& quat)
    {
        quat_ = Eigen::Quaterniond(quat[3], quat[1], quat[2], quat[0]);
    }
    Rotation(geometry_msgs::msg::Quaternion const& quat)
    {
        quat_ = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
    }
    Rotation(geometry_msgs::msg::Vector3 const& rot_vec)
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
    Rotation()
    {
        quat_ = Eigen::Quaterniond(1, 0, 0, 0);
    }
    double roll() const
    {
        return rot_vec()[0];
    }
    double pitch() const
    {
        return rot_vec()[1];
    }
    double yaw() const
    {
        return rot_vec()[2];
    }
    inline double quat_w() const
    {
        return quat_.w();
    }
    inline double quat_x() const
    {
        return quat_.x();
    }
    inline double quat_y() const
    {
        return quat_.y();
    }
    inline double quat_z() const
    {
        return quat_.z();
    }
    inline double norm() const
    {
        return quat_.norm();
    }
    inline double roll_deg() const
    {
        return roll() * 180 / M_PI;
    }
    inline double pitch_deg() const
    {
        return pitch() * 180 / M_PI;
    }
    inline double yaw_deg() const
    {
        return yaw() * 180 / M_PI;
    }
    Eigen::Vector3d rot_vec(Axis const& first = Axis::X, Axis const& second = Axis::Y,
                            Axis const& third = Axis::Z) const
    {
        return quat_.toRotationMatrix().eulerAngles(static_cast<int>(first), static_cast<int>(second),
                                                    static_cast<int>(third));
    }
    std::tuple<double, double, double> rot_vec_tuple(Axis const& first = Axis::X, Axis const& second = Axis::Y,
                                                     Axis const& third = Axis::Z) const
    {
        Eigen::Vector3d rot_vec_eigen{ rot_vec(first, second, third) };
        return { rot_vec_eigen[0], rot_vec_eigen[1], rot_vec_eigen[2] };
    }
    Eigen::Matrix3d rot_mat() const
    {
        Eigen::Matrix3d rot_mat = quat_.toRotationMatrix();
        return rot_mat;
    }
    inline Eigen::Quaterniond quat() const
    {
        return quat_;
    }
    geometry_msgs::msg::Quaternion quat_msg() const
    {
        geometry_msgs::msg::Quaternion quat;
        quat.w = quat_.w();
        quat.x = quat_.x();
        quat.y = quat_.y();
        quat.z = quat_.z();
        return quat;
    }
    geometry_msgs::msg::Vector3 rot_vec_msg() const
    {
        geometry_msgs::msg::Vector3 res;
        Eigen::Vector3d rot_vec_eigen{ rot_vec() };
        res.x = rot_vec_eigen[0];
        res.y = rot_vec_eigen[1];
        res.z = rot_vec_eigen[2];
        return res;
    }
    Eigen::Vector4d quat_vec() const
    {
        Eigen::Vector4d quat_vec;
        quat_vec << quat_.w(), quat_.x(), quat_.y(), quat_.z();
        return quat_vec;
    }
    bool operator==(Rotation const& other) const
    {
        return quat_.w() == other.quat_w() && quat_.x() == other.quat_x() && quat_.y() == other.quat_y() &&
               quat_.z() == other.quat_z();
    }
};

}  // namespace mil_tools::geometry
