// Compile-and-construct check for constants.hpp. Nothing else pulls this
// header into a real target until Task 7, so without this a break in it would
// surface several tasks later, far from the change that caused it.

#include <gtest/gtest.h>

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/constants.hpp"

namespace
{
class Fixture : public ::testing::Test
{
  protected:
    static void SetUpTestSuite()
    {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite()
    {
        rclcpp::shutdown();
    }
};
}  // namespace

TEST_F(Fixture, LoadsWithDefaults)
{
    auto node = std::make_shared<rclcpp::Node>("constants_defaults");
    prop_maneuvers::Constants settings(node.get());
    EXPECT_GT(settings.circle_radius_, 0.0);
    EXPECT_GE(settings.circle_legs_, 3);
    EXPECT_GT(settings.max_turn_rate_, 0.0);
}

// The shipped config is what actually runs, and nothing else loads it. Both
// guards in Constants -- the odd-length check and the backwards-pair check --
// are fatal at construction, so a typo in maneuvers.yaml surfaces as a node
// that dies on the water rather than as a red test. A backwards pair has
// already happened once; it turned a 45 degree blind spot into a 315 degree
// one and looked like a dead lidar.
TEST_F(Fixture, TheShippedConfigLoads)
{
    rclcpp::NodeOptions options;
    options.arguments({ "--ros-args", "--params-file", MANEUVERS_YAML });
    auto node = std::make_shared<rclcpp::Node>("constants_shipped", options);
    prop_maneuvers::Constants settings(node.get());

    // Two mirrored antenna wedges, one per side.
    EXPECT_EQ(settings.blind_spots_.size(), 2u);

    // Settling, used by ApproachObject to wait for the boat to actually stop
    // before it reports where it ended up. Both must be positive or the wait
    // either never ends or never happens.
    EXPECT_GT(settings.stop_speed_, 0.0);
    EXPECT_GT(settings.stop_timeout_, 0.0);
}

TEST_F(Fixture, RefusesAMalformedBlindSpotList)
{
    rclcpp::NodeOptions options;
    // Odd number of values: a typo, and the quiet path would leave the Task 10
    // mask masking nothing while appearing to pass.
    options.parameter_overrides({ { "blind_spots_deg", std::vector<double>{ 90.0, 160.0, -90.0 } } });
    auto node = std::make_shared<rclcpp::Node>("constants_malformed", options);
    EXPECT_THROW(prop_maneuvers::Constants settings(node.get()), std::runtime_error);
}
