#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <filesystem>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

#include "actuate_servo.hpp"
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
#include <std_srvs/srv/set_bool.hpp>
#include <topic_ticker.hpp>
#include <yaw_style.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mission_planner");
    auto ctx = std::make_shared<Context>();
    ctx->node = node;

    // INSERT TEST CODE FOR GRIPPER CONTROL PLUGIN HERE

    ctx->gripper_client = node->create_client<std_srvs::srv::SetBool>("gripper_control/set_open");
    while (!ctx->gripper_client->wait_for_service(std::chrono::seconds(1)))
    {
    };

    // actuateServo(node, ctx->gripper_client, true);

    // for (int i = 0; i < 5; i++){
    //     actuateServo(node, ctx->gripper_client, false);
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    //     actuateServo(node, ctx->gripper_client, true);
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }

    // node->declare_parameter<std::string>("mission", "SonarFollowerTest");
    // std::string mission_to_run = node->get_parameter("mission").as_string();

    // // Topics to subscribe/publish to
    // ctx->goal_pub = node->create_publisher<geometry_msgs::msg::Pose>("/goal_pose", 10);
    // ctx->odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10,
    //                                                                    [ctx](nav_msgs::msg::Odometry::SharedPtr msg)
    //                                                                    {
    //                                                                        std::scoped_lock lk(ctx->odom_mx);
    //                                                                        ctx->latest_odom = *msg;
    //                                                                    });

    // // Perception targets: from your YOLO node
    // ctx->targets_sub =
    //     node->create_subscription<yolo_msgs::msg::DetectionArray>("/yolo/detections", 10,
    //                                                               [ctx](yolo_msgs::msg::DetectionArray::SharedPtr
    //                                                               msg)
    //                                                               {
    //                                                                   std::scoped_lock lk(ctx->detections_mx);
    //                                                                   ctx->latest_detections = *msg;
    //                                                               });

    // // Image size (for pixel->angle mapping). Probably do not need
    // ctx->image_sub = node->create_subscription<sensor_msgs::msg::Image>("/front_cam/image_raw", 10,
    //                                                                     [ctx](sensor_msgs::msg::Image::SharedPtr msg)
    //                                                                     {
    //                                                                         std::scoped_lock lk(ctx->img_mx);
    //                                                                         ctx->img_width = msg->width;
    //                                                                         ctx->img_height = msg->height;
    //                                                                     });

    // // Wait for odometry before starting mission
    // RCLCPP_INFO(node->get_logger(), "Waiting for odometry...");
    // rclcpp::Rate wait_rate(10.0);
    // while (rclcpp::ok())
    // {
    //     rclcpp::spin_some(node);
    //     {
    //         std::scoped_lock lk(ctx->odom_mx);
    //         if (ctx->latest_odom.has_value())
    //         {
    //             RCLCPP_INFO(node->get_logger(), "Odometry received. Starting mission!");
    //             break;
    //         }
    //     }
    //     wait_rate.sleep();
    // }

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ActuateServo>("ActuateServo");
    // Load and run mission by name. Default mission is TestServos which
    // exercises the `ActuateServo` node for servo IDs 1,2,3 toggling true/false.
    node->declare_parameter<std::string>("mission", "TestServos");
    std::string mission_to_run = node->get_parameter("mission").as_string();

    std::string const pkg_share = ament_index_cpp::get_package_share_directory("mission_planner");
    std::string const bt_dir = (std::filesystem::path(pkg_share) / "subjugator_missions" / "xml").string();
    auto bt_path = [&](std::string const& file) { return (std::filesystem::path(bt_dir) / file).string(); };

    std::string mission_file = mission_to_run + ".xml";
    std::string fallback_file = "test_servos.xml";

    try
    {
        std::string file_to_load = bt_path(mission_file);
        if (!std::filesystem::exists(file_to_load))
        {
            file_to_load = bt_path(fallback_file);
        }
        // If not found in installed package share, try to locate the file
        // somewhere in the workspace (useful when running from source).
        if (!std::filesystem::exists(file_to_load))
        {
            for (auto const& entry : std::filesystem::recursive_directory_iterator(std::filesystem::current_path()))
            {
                if (!entry.is_regular_file())
                    continue;
                if (entry.path().filename() == fallback_file)
                {
                    file_to_load = entry.path().string();
                    break;
                }
            }
        }

        if (!std::filesystem::exists(file_to_load))
        {
            RCLCPP_FATAL(node->get_logger(), "Behavior tree file not found: %s", file_to_load.c_str());
            rclcpp::shutdown();
            return 1;
        }

        factory.registerBehaviorTreeFromFile(file_to_load);
    }
    catch (std::exception const& e)
    {
        RCLCPP_FATAL(node->get_logger(), "Failed to load behavior tree file: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

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

    BT::Groot2Publisher publisher(*tree_ptr);
    BT::StdCoutLogger logger_cout(*tree_ptr);

    RCLCPP_INFO(node->get_logger(), "Mission Planner started. Ticking tree…");
    rclcpp::WallRate rate(20.0);

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

    // END TEST CODE

    // factory.registerNodeType<PublishGoalPose>("PublishGoalPose");
    // factory.registerNodeType<AtGoalPose>("AtGoalPose");
    // factory.registerNodeType<DetectTarget>("DetectTarget");
    // factory.registerNodeType<HoneBearing>("HoneBearing");
    // factory.registerNodeType<CheckYoloModel>("CheckYoloModel");
    // factory.registerNodeType<TrackLargestPoles>("TrackLargestPoles");
    // factory.registerNodeType<PolesBigEnough>("PolesBigEnough");
    // factory.registerNodeType<DetermineChannelSide>("DetermineChannelSide");
    // factory.registerNodeType<AnyPolesDetected>("AnyPolesDetected");
    // factory.registerNodeType<HasFoundPair>("HasFoundPair");

    // factory.registerNodeType<TopicTicker<nav_msgs::msg::Odometry>>("TopicTicker");
    // factory.registerNodeType<CountWhenTicked>("CountWhenTicked");
    // factory.registerNodeType<SonarFollower>("SonarFollower");
    // factory.registerNodeType<YawStyle>("YawStyle");

    // // Load all tree models from installed xml
    // std::string const pkg_share = ament_index_cpp::get_package_share_directory("mission_planner");
    // std::string const bt_dir = (std::filesystem::path(pkg_share) / "bt").string();
    // auto bt_path = [&](std::string const& file) { return (std::filesystem::path(bt_dir) / file).string(); };
    // factory.registerBehaviorTreeFromFile(bt_path("sub9_missions.xml"));

    // // Create by name
    // auto blackboard = BT::Blackboard::create();
    // blackboard->set("ctx", ctx);

    // std::unique_ptr<BT::Tree> tree_ptr;
    // try
    // {
    //     tree_ptr = std::make_unique<BT::Tree>(factory.createTree(mission_to_run, blackboard));
    // }
    // catch (std::exception const& e)
    // {
    //     RCLCPP_FATAL(node->get_logger(), "Unknown mission '%s' Error: %s", mission_to_run.c_str(), e.what());
    //     rclcpp::shutdown();
    //     return 1;
    // }

    // // For live feed of tree
    // BT::Groot2Publisher publisher(*tree_ptr);

    // // Log BT transitions to console
    // BT::StdCoutLogger logger_cout(*tree_ptr);

    // RCLCPP_INFO(node->get_logger(), "Mission Planner started. Ticking tree…");
    // rclcpp::WallRate rate(20.0);

    // std::string xml_models = BT::writeTreeNodesModelXML(factory);
    // std::ofstream("/home/carlos/models.xml") << xml_models;

    // while (rclcpp::ok())
    // {
    //     rclcpp::spin_some(node);
    //     BT::NodeStatus status = tree_ptr->tickOnce();

    //     if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
    //     {
    //         RCLCPP_INFO(node->get_logger(), "Mission finished with status: %s. Shutting down.",
    //                     (status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE"));
    //         tree_ptr->haltTree();
    //         break;
    //     }
    //     rate.sleep();
    // }

    rclcpp::shutdown();
    return 0;
}
