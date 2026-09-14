/**
 * @file blind_spot_mask_node.cpp
 * @brief Drops the angles the real lidar cannot see, out of a simulated cloud.
 *
 * TEST SUPPORT ONLY. The simulated lidar sees a full circle; on the real boat
 * the two antennas standing beside it shadow a wedge of about 25 degrees on
 * each side, near abeam. See blind_spots_deg in config/maneuvers.yaml.
 * Masking here, before the clustering sees the cloud, tests the whole chain
 * rather than just the maneuver code.
 *
 *   in   cloud         sensor_msgs/PointCloud2
 *   out  masked_cloud  sensor_msgs/PointCloud2, blocked angles removed
 *
 * THIS MASKS THE CLOUD, NOT THE SCAN, and that is the whole point. It used to
 * mask sensor_msgs/LaserScan, which was right when the simulated boat carried
 * a single flat ring and the maneuvers clustered on a scan converted to a
 * cloud. Since Carlos's #577 the boat carries a real 16-beam VLP-16 and the
 * ros_gz bridge publishes its cloud directly, so nothing downstream reads the
 * scan any more -- and this node was masking a topic with no consumers.
 * Measured 2026-09-13: the bridged LaserScan is the MIDDLE of the sixteen
 * rings, about a degree above horizontal, and passes clean over the 0.5 m
 * buoys -- a median of 0 finite returns out of 1875. So the mask ran, reported
 * its 312 readings, and changed nothing that perception ever saw.
 *
 * Points are dropped rather than pushed to infinity: a cloud has no way to say
 * "nothing came back along this ray", which is the one thing a scan's infinity
 * could express. Downstream only ever counts points, so the two are equivalent
 * to every consumer there is.
 *
 * The blind spot angles come from the SAME setting the maneuvers read
 * (blind_spots_deg in config/maneuvers.yaml), so there is one place to change
 * if the lidar moves.
 *
 * A point's bearing here is measured from the front of the boat with left
 * positive, and prop.urdf mounts the lidar with no rotation relative to the
 * hull, so cloud bearings and blind spot angles are directly comparable.
 */

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/constants.hpp"
#include "prop_maneuvers/geometry.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace prop_maneuvers;

namespace
{

/// Byte offset of a named FLOAT32 field, or -1 if it is missing or the wrong type.
int float_field_offset(sensor_msgs::msg::PointCloud2 const &cloud, std::string const &name)
{
    for (auto const &field : cloud.fields)
    {
        if (field.name == name)
        {
            return field.datatype == sensor_msgs::msg::PointField::FLOAT32 ? static_cast<int>(field.offset) : -1;
        }
    }
    return -1;
}

}  // namespace

class BlindSpotMask : public rclcpp::Node, public Constants
{
  public:
    BlindSpotMask() : rclcpp::Node("blind_spot_mask"), Constants(this)
    {
        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("masked_cloud", rclcpp::SensorDataQoS());

        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr const msg) { mask(*msg); });
    }

  private:
    void mask(sensor_msgs::msg::PointCloud2 const &in)
    {
        int const off_x = float_field_offset(in, "x");
        int const off_y = float_field_offset(in, "y");
        if (off_x < 0 || off_y < 0 || in.point_step == 0)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "cloud has no float x/y fields to take a bearing from; passing it through");
            publisher_->publish(in);
            return;
        }

        std::size_t const stride = in.point_step;
        std::size_t const total = in.data.size() / stride;

        // Copy whole points as opaque bytes, so whatever else the cloud
        // carries alongside x/y/z survives untouched.
        sensor_msgs::msg::PointCloud2 out;
        out.header = in.header;
        out.fields = in.fields;
        out.is_bigendian = in.is_bigendian;
        out.point_step = in.point_step;
        out.is_dense = in.is_dense;
        out.data.reserve(in.data.size());

        std::size_t kept = 0;
        for (std::size_t i = 0; i < total; ++i)
        {
            uint8_t const *point = in.data.data() + i * stride;
            float x = 0.0F;
            float y = 0.0F;
            std::memcpy(&x, point + off_x, sizeof(float));
            std::memcpy(&y, point + off_y, sizeof(float));

            // A ray that returned nothing has no bearing worth judging; leave
            // it for the range filter downstream.
            bool const has_bearing = std::isfinite(x) && std::isfinite(y);
            if (has_bearing && use_blind_spots_ && is_blind(wrap_angle(std::atan2(y, x)), blind_spots_))
            {
                continue;
            }

            out.data.insert(out.data.end(), point, point + stride);
            ++kept;
        }

        out.width = static_cast<uint32_t>(kept);
        out.height = 1;
        out.row_step = static_cast<uint32_t>(kept * stride);

        RCLCPP_INFO_ONCE(get_logger(), "masking %zu of %zu points per cloud", total - kept, total);
        publisher_->publish(out);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BlindSpotMask>());
    rclcpp::shutdown();
    return 0;
}
