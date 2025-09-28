#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

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

    // Build the tree
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<PublishGoalPose>("PublishGoalPose");
    factory.registerNodeType<AtGoalPose>("AtGoalPose");

    MovementMission mission;
    MissionParams params{};  // Does nothing for now
    std::string const xml = mission.buildTreeXml(params);
    factory.registerBehaviorTreeFromText(xml);

    RelativeMotionMission rel_subtree;
    factory.registerBehaviorTreeFromText(rel_subtree.buildTreeXml(params));

    auto blackboard = BT::Blackboard::create();
    blackboard->set("ctx", ctx);

    // Create main tree from the square mission XML
    SquareTestMission square;
    auto square_xml = square.buildTreeXml(params);
    auto tree = factory.createTreeFromText(square_xml, blackboard);  // now <SubTree ID="RelativeMove"> resolves

    // Log BT transitions to console (pretty neat)
    BT::StdCoutLogger logger_cout(tree);

    RCLCPP_INFO(node->get_logger(), "Mission Planner started. Ticking tree…");

    rclcpp::WallRate rate(20.0);  // Might be worth tuning
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        BT::NodeStatus status = tree.tickRoot();

        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
        {
            RCLCPP_INFO(node->get_logger(), "Mission finished with status: %s. Shutting down.",
                        (status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE"));
            tree.haltTree();
            break;  // exit the loop and shut down the node/process
        }
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
