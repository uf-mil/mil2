/**
 * @file blind_spot_mask_node.cpp
 * @brief Drops the angles the real lidar cannot see, out of a simulated scan.
 *
 * TEST SUPPORT ONLY. The simulated lidar sees a full circle; on the real boat
 * the two antennas standing beside it shadow a wedge of about 25 degrees on
 * each side, near abeam. See blind_spots_deg in config/maneuvers.yaml.
 * Masking here, before the clustering sees the scan, tests the whole chain
 * rather than just the maneuver code.
 *
 *   in   scan         sensor_msgs/LaserScan
 *   out  masked_scan  sensor_msgs/LaserScan, blocked angles set to infinity
 *
 * The blind spot angles come from the SAME setting the maneuvers read
 * (blind_spots_deg in config/maneuvers.yaml), so there is one place to change
 * if the lidar moves.
 *
 * A scan angle is already measured from the front of the boat with left
 * positive, and prop.urdf mounts the lidar with no rotation relative to the
 * hull, so scan angles and blind spot angles are directly comparable.
 */

#include <cmath>
#include <limits>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/constants.hpp"
#include "prop_maneuvers/geometry.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

using namespace prop_maneuvers;

class BlindSpotMask : public rclcpp::Node, public Constants
{
  public:
    BlindSpotMask() : rclcpp::Node("blind_spot_mask"), Constants(this)
    {
        publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("masked_scan", rclcpp::SensorDataQoS());

        subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr const msg)
            {
                sensor_msgs::msg::LaserScan masked = *msg;
                std::size_t dropped = 0;

                for (std::size_t i = 0; i < masked.ranges.size(); ++i)
                {
                    double const angle = masked.angle_min + static_cast<double>(i) * masked.angle_increment;
                    if (use_blind_spots_ && is_blind(wrap_angle(angle), blind_spots_))
                    {
                        masked.ranges[i] = std::numeric_limits<float>::infinity();
                        ++dropped;
                    }
                }

                RCLCPP_INFO_ONCE(get_logger(), "masking %zu of %zu readings per scan", dropped, masked.ranges.size());
                publisher_->publish(masked);
            });
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BlindSpotMask>());
    rclcpp::shutdown();
    return 0;
}
