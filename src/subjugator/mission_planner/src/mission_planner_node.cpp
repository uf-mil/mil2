#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <fstream>

#include <rclcpp/rclcpp.hpp>

#include "any_poles_detected.hpp"
#include "at_goal_pose.hpp"
#include "check_yolo_model.hpp"
#include "context.hpp"
#include "detect_target.hpp"
#include "determine_channel_side.hpp"
#include "has_found_pair.hpp"
#include "hone_bearing.hpp"
#include "poles_big_enough.hpp"
#include "publish_goal.hpp"
#include "track_largest_poles.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <count_when_ticked.hpp>
#include <go_to_pinger.hpp>
#include <topic_ticker.hpp>
#include <yaw_style.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mission_planner");
    auto ctx = std::make_shared<Context>();
    ctx->node = node;

    node->declare_parameter<std::string>("mission", "YawStyleTest");
    std::string mission_to_run = node->get_parameter("mission").as_string();

    // Topics to subscribe/publish to
    ctx->goal_pub = node->create_publisher<geometry_msgs::msg::Pose>("/goal_pose", 10);
    ctx->odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10,
                                                                       [ctx](nav_msgs::msg::Odometry::SharedPtr msg)
                                                                       {
                                                                           std::scoped_lock lk(ctx->odom_mx);
                                                                           ctx->latest_odom = *msg;
                                                                       });

    // Perception targets: from your YOLO node
    ctx->targets_sub =
        node->create_subscription<yolo_msgs::msg::DetectionArray>("/yolo/detections", 10,
                                                                  [ctx](yolo_msgs::msg::DetectionArray::SharedPtr msg)
                                                                  {
                                                                      std::scoped_lock lk(ctx->detections_mx);
                                                                      ctx->latest_detections = *msg;
                                                                  });

    // Image size (for pixel->angle mapping). Probably do not need
    ctx->image_sub = node->create_subscription<sensor_msgs::msg::Image>("/front_cam/image_raw", 10,
                                                                        [ctx](sensor_msgs::msg::Image::SharedPtr msg)
                                                                        {
                                                                            std::scoped_lock lk(ctx->img_mx);
                                                                            ctx->img_width = msg->width;
                                                                            ctx->img_height = msg->height;
                                                                        });

    // Wait for odometry before starting mission
    RCLCPP_INFO(node->get_logger(), "Waiting for odometry...");
    rclcpp::Rate wait_rate(10.0);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        {
            std::scoped_lock lk(ctx->odom_mx);
            if (ctx->latest_odom.has_value())
            {
                RCLCPP_INFO(node->get_logger(), "Odometry received. Starting mission!");
                break;
            }
        }
        wait_rate.sleep();
    }

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<PublishGoalPose>("PublishGoalPose");
    factory.registerNodeType<AtGoalPose>("AtGoalPose");
    factory.registerNodeType<DetectTarget>("DetectTarget");
    factory.registerNodeType<HoneBearing>("HoneBearing");
    factory.registerNodeType<CheckYoloModel>("CheckYoloModel");
    factory.registerNodeType<TrackLargestPoles>("TrackLargestPoles");
    factory.registerNodeType<PolesBigEnough>("PolesBigEnough");
    factory.registerNodeType<DetermineChannelSide>("DetermineChannelSide");
    factory.registerNodeType<AnyPolesDetected>("AnyPolesDetected");
    factory.registerNodeType<HasFoundPair>("HasFoundPair");

    factory.registerNodeType<TopicTicker<nav_msgs::msg::Odometry>>("TopicTicker");
    factory.registerNodeType<CountWhenTicked>("CountWhenTicked");
    factory.registerNodeType<SonarFollower>("SonarFollower");
    factory.registerNodeType<YawStyle>("YawStyle");

    // Load all tree models from installed xml
    std::string const pkg_share = ament_index_cpp::get_package_share_directory("mission_planner");
    std::string const bt_dir = (std::filesystem::path(pkg_share) / "bt").string();
    auto bt_path = [&](std::string const& file) { return (std::filesystem::path(bt_dir) / file).string(); };
    factory.registerBehaviorTreeFromFile(bt_path("sub9_missions.xml"));

    // Create by name
    auto blackboard = BT::Blackboard::create();
    blackboard->set("ctx", ctx);

    std::unique_ptr<BT::Tree> tree_ptr;
    try
    {
        tree_ptr = std::make_unique<BT::Tree>(factory.createTree(mission_to_run, blackboard));
    }
    catch (std::exception const& e)
    {
        RCLCPP_FATAL(node->get_logger(), "Unknown mission '%s' Error: %s", mission_to_run.c_str(), e.what());
        rclcpp::shutdown();
        return 1;
    }

    // For live feed of tree
    BT::Groot2Publisher publisher(*tree_ptr);

    // Log BT transitions to console
    BT::StdCoutLogger logger_cout(*tree_ptr);

    RCLCPP_INFO(node->get_logger(), "Mission Planner started. Ticking tree…");
    rclcpp::WallRate rate(20.0);

    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    std::ofstream("/home/carlos/models.xml") << xml_models;

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        BT::NodeStatus status = tree_ptr->tickOnce();

        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
        {
            RCLCPP_INFO(node->get_logger(), "Mission finished with status: %s. Shutting down.",
                        (status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE"));
            tree_ptr->haltTree();
            break;
        }
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
