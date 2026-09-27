/**
 * @file mission_planner_node.cpp
 * @brief Runs one boat mission from missions/prop_missions.xml.
 *
 *   bmp BuoyTour --sim
 *   ros2 run prop_mission_planner mission_planner_node --ros-args \
 *       --params-file <prop_maneuvers share>/config/maneuvers.yaml -p mission:=BuoyTour
 *
 * Waits for a position estimate, builds the named tree and ticks it at 10 Hz
 * until it finishes. However it ends -- success, failure, an exception, or
 * Ctrl-C -- the tree is halted and the motors are told to stop BEFORE ROS
 * shuts down.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "prop_mission_planner/context.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace
{

std::atomic<bool> stop_requested{ false };

void request_stop(int /*signal*/)
{
    stop_requested = true;
}

std::string join(std::vector<std::string> const &items)
{
    std::string out;
    for (auto const &item : items)
    {
        out += (out.empty() ? "" : ", ") + item;
    }
    return out;
}

}  // namespace

int main(int argc, char **argv)
{
    // Our own signal handling. rclcpp's handler shuts the context down the
    // moment Ctrl-C arrives, before the loop below notices -- and then the stop
    // messages at the end could not be published, leaving guidance following
    // its last path.
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);

    auto node = rclcpp::Node::make_shared("prop_mission_planner");
    auto const mission = node->declare_parameter<std::string>("mission", "FaceTest");
    auto const models_xml = node->declare_parameter<std::string>("models_xml", "");
    auto ctx = prop_mission_planner::make_context(node);
    auto &factory = prop_mission_planner::factory();

    RCLCPP_INFO(ctx->logger(), "waiting for a position estimate on odometry/filtered/global");
    rclcpp::Clock steady(RCL_STEADY_TIME);
    while (!stop_requested && rclcpp::ok() && !ctx->maneuvers->boat().valid)
    {
        rclcpp::spin_some(node);
        RCLCPP_INFO_THROTTLE(ctx->logger(), steady, 5000, "still waiting for a position estimate");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // Under use_sim_time the clock reads 0 until the first /clock message, and
    // /clock is handled on its own thread, unordered with odometry. A
    // RosTimeout armed at t=0 would expire the moment sim time arrives, so
    // wait for a real time as well as a position.
    while (!stop_requested && rclcpp::ok() && node->now().nanoseconds() == 0)
    {
        rclcpp::spin_some(node);
        RCLCPP_INFO_THROTTLE(ctx->logger(), steady, 5000, "waiting for the clock (use_sim_time with no /clock?)");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (stop_requested || !rclcpp::ok())
    {
        rclcpp::shutdown();
        return 1;
    }

    auto blackboard = BT::Blackboard::create();
    blackboard->set("ctx", ctx);
    std::unique_ptr<BT::Tree> tree;
    try
    {
        auto const share = ament_index_cpp::get_package_share_directory("prop_mission_planner");
        factory.registerBehaviorTreeFromFile(share + "/bt/prop_missions.xml");
        tree = std::make_unique<BT::Tree>(factory.createTree(mission, blackboard));
    }
    catch (std::exception const &error)
    {
        RCLCPP_FATAL(ctx->logger(), "cannot start mission '%s': %s", mission.c_str(), error.what());
        RCLCPP_FATAL(ctx->logger(), "known missions: %s", join(factory.registeredBehaviorTrees()).c_str());
        rclcpp::shutdown();
        return 1;
    }

    if (!models_xml.empty())
    {
        std::ofstream(models_xml) << BT::writeTreeNodesModelXML(factory);
    }

    BT::Groot2Publisher groot(*tree);
    BT::StdCoutLogger console(*tree);

    RCLCPP_INFO(ctx->logger(), "running mission '%s'", mission.c_str());
    auto status = BT::NodeStatus::RUNNING;
    // Wall-clock pacing so Ctrl-C is answered even while a sim is paused.
    // Everything inside the tree times itself with node->now(), so a paused
    // sim still pauses the mission.
    auto next_tick = std::chrono::steady_clock::now();
    try
    {
        while (!stop_requested && rclcpp::ok())
        {
            rclcpp::spin_some(node);
            status = tree->tickOnce();
            if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
            {
                break;
            }
            next_tick += std::chrono::milliseconds(100);
            std::this_thread::sleep_until(next_tick);
        }
    }
    catch (std::exception const &error)
    {
        RCLCPP_FATAL(ctx->logger(), "mission '%s' threw: %s", mission.c_str(), error.what());
        status = BT::NodeStatus::FAILURE;
    }

    if (stop_requested)
    {
        RCLCPP_WARN(ctx->logger(), "interrupted; stopping the boat");
    }
    else
    {
        RCLCPP_INFO(ctx->logger(), "mission '%s' finished: %s", mission.c_str(),
                    status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
    }
    tree->haltTree();
    ctx->stop_motors();
    rclcpp::shutdown();
    return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
