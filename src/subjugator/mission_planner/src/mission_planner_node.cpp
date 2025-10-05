#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <fstream>

#include <rclcpp/rclcpp.hpp>

#include "at_goal_pose.hpp"
#include "context.hpp"
#include "missions.hpp"
#include "publish_goal.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mission_planner");
    auto ctx = std::make_shared<Context>();
    ctx->node = node;

    // Topics to subscribe/publish to
    ctx->goal_pub = node->create_publisher<geometry_msgs::msg::Pose>("/goal_pose", 10);
    ctx->odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10,
                                                                       [ctx](nav_msgs::msg::Odometry::SharedPtr msg)
                                                                       {
                                                                           std::scoped_lock lk(ctx->odom_mx);
                                                                           ctx->latest_odom = *msg;
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
                RCLCPP_INFO(node->get_logger(), "Odometry received. Starting mission...");
                break;
            }
        }
        wait_rate.sleep();
    }

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<PublishGoalPose>("PublishGoalPose");
    factory.registerNodeType<AtGoalPose>("AtGoalPose");

    MissionParams params{};

    // Register subtree first
    RelativeMotionMission rel_subtree;
    factory.registerBehaviorTreeFromText(rel_subtree.buildTreeXml(params));

    // Register main tree
    SquareTestMission square;
    factory.registerBehaviorTreeFromText(square.buildTreeXml(params));

    // Create by name
    auto blackboard = BT::Blackboard::create();
    blackboard->set("ctx", ctx);
    auto tree = factory.createTree("SquareTestMission", blackboard);

    // For live feed of tree
    BT::Groot2Publisher publisher(tree);

    // Log BT transitions to console
    BT::StdCoutLogger logger_cout(tree);

    RCLCPP_INFO(node->get_logger(), "Mission Planner started. Ticking tree…");
    rclcpp::WallRate rate(20.0);

    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    std::ofstream("/home/carlos/models.xml") << xml_models;

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        BT::NodeStatus status = tree.tickOnce();

        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
        {
            RCLCPP_INFO(node->get_logger(), "Mission finished with status: %s. Shutting down.",
                        (status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE"));
            tree.haltTree();
            break;
        }
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
