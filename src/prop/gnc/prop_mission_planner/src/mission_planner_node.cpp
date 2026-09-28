/**
 * @file mission_planner_node.cpp
 * @brief Runs one boat mission from missions/prop_missions.xml.
 *
 *   bmp BuoyTour --sim
 *   ros2 run prop_mission_planner mission_planner_node --ros-args \
 *       --params-file <prop_maneuvers share>/config/maneuvers.yaml -p mission:=BuoyTour
 *
 * Builds the named tree first, so a bad mission name fails at once, then waits
 * for a fresh position estimate and a clock, and ticks the tree at 10 Hz until
 * it finishes. However it ends -- success, failure, an exception, or
 * Ctrl-C -- the tree is halted and the motors are told to stop BEFORE ROS
 * shuts down. Both of those cleanup steps run even if the other one throws.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
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
// request_stop reads-then-sets this flag with no lock; that is only safe to
// do from a signal handler if the type is lock-free.
static_assert(std::atomic<bool>::is_always_lock_free, "the signal handler relies on a lock-free flag");

void request_stop(int signal)
{
    // First Ctrl-C: ask the main loop to stop cleanly (halt the tree, stop the
    // motors, then shut down). A second one means the clean path is stuck or
    // too slow, so restore the default handler and re-raise: this process
    // dies right here, the way it would have without our handler at all.
    if (stop_requested.exchange(true))
    {
        std::signal(signal, SIG_DFL);
        std::raise(signal);
    }
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
    auto const logger = node->get_logger();
    auto &factory = prop_mission_planner::factory();

    // A parameter of the wrong type (-p mission:=5) throws from here, so these
    // are caught and reported rather than left to terminate() the process.
    std::string mission;
    std::string models_xml;
    std::int64_t groot_port = 0;
    try
    {
        // No default: running "the default mission" by accident moves the boat.
        mission = node->declare_parameter<std::string>("mission", "");
        models_xml = node->declare_parameter<std::string>("models_xml", "");
        groot_port = node->declare_parameter<std::int64_t>("groot_port", 1667);
    }
    catch (std::exception const &error)
    {
        RCLCPP_FATAL(logger, "bad parameter: %s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    try
    {
        auto const share = ament_index_cpp::get_package_share_directory("prop_mission_planner");
        factory.registerBehaviorTreeFromFile(share + "/bt/prop_missions.xml");
    }
    catch (std::exception const &error)
    {
        RCLCPP_FATAL(logger, "cannot load the mission index: %s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    if (mission.empty())
    {
        RCLCPP_FATAL(logger, "no mission given (-p mission:=<name>); known missions: %s",
                     join(factory.registeredBehaviorTrees()).c_str());
        rclcpp::shutdown();
        return 1;
    }

    // Declares the maneuver parameters: Constants throws on a malformed
    // blind_spots_deg, and a wrongly typed value throws too. Nothing in here
    // publishes, so failing now moves nothing.
    std::shared_ptr<prop_mission_planner::Context> ctx;
    try
    {
        ctx = prop_mission_planner::make_context(node);
    }
    catch (std::exception const &error)
    {
        RCLCPP_FATAL(logger, "cannot set up the maneuvers: %s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    // The tree is built BEFORE waiting for odometry and the clock, so a bad
    // mission name or a malformed mission fails at once rather than after the
    // stack comes up. Building ticks nothing; the first tick is after the wait.
    auto blackboard = BT::Blackboard::create();
    blackboard->set("ctx", ctx);
    std::unique_ptr<BT::Tree> tree;
    try
    {
        tree = std::make_unique<BT::Tree>(factory.createTree(mission, blackboard));
    }
    catch (std::exception const &error)
    {
        RCLCPP_FATAL(logger, "cannot start mission '%s': %s", mission.c_str(), error.what());
        RCLCPP_FATAL(logger, "known missions: %s", join(factory.registeredBehaviorTrees()).c_str());
        rclcpp::shutdown();
        return 1;
    }

    if (!models_xml.empty())
    {
        std::ofstream out(models_xml);
        out << BT::writeTreeNodesModelXML(factory);
        if (!out)
        {
            RCLCPP_WARN(logger, "could not write the node models to '%s'", models_xml.c_str());
        }
    }

    // A busy port (e.g. another mission planner already running) must not
    // abort the mission: Groot2 is a debugging aid, not a dependency. Neither
    // does an out-of-range port: Groot2Publisher takes an unsigned, so a
    // negative value would otherwise wrap around into something in range.
    // 65535 is out too, because Groot2 binds port AND port+1.
    std::unique_ptr<BT::Groot2Publisher> groot;
    if (groot_port != 0 && (groot_port < 0 || groot_port > 65534))
    {
        RCLCPP_WARN(logger,
                    "invalid groot_port %ld: use 1 to 65534 (Groot2 also binds port+1), or 0 for off; Groot2 disabled",
                    static_cast<long>(groot_port));
    }
    else if (groot_port != 0)
    {
        try
        {
            groot = std::make_unique<BT::Groot2Publisher>(*tree, static_cast<unsigned>(groot_port));
        }
        catch (std::exception const &error)
        {
            RCLCPP_WARN(logger, "Groot2 disabled: %s (is another mission planner running?)", error.what());
        }
    }
    BT::StdCoutLogger console(*tree);

    // One executor for the whole run. rclcpp::spin_some(node) builds and tears
    // down a temporary executor on every call, ten times a second.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Wait for a FRESH position estimate and a real time, together. Under
    // use_sim_time the clock reads 0 until the first /clock message, and /clock
    // is handled on its own thread, unordered with odometry: a RosTimeout armed
    // at t=0 would expire the moment sim time arrives, and odometry received
    // before the clock looks seconds old once it does.
    rclcpp::Clock steady(RCL_STEADY_TIME);
    RCLCPP_INFO(logger, "waiting for a position estimate on odometry/filtered/global");
    try
    {
        while (!stop_requested && rclcpp::ok())
        {
            executor.spin_some();
            bool const have_clock = node->now().nanoseconds() != 0;
            double const odometry_age = ctx->maneuvers->odometry_age();
            if (have_clock && odometry_age <= ctx->settings->max_odometry_age_)
            {
                break;
            }
            if (!have_clock)
            {
                RCLCPP_INFO_THROTTLE(logger, steady, 5000, "waiting for the clock (use_sim_time with no /clock?)");
            }
            else if (!ctx->maneuvers->boat().valid)
            {
                RCLCPP_INFO_THROTTLE(logger, steady, 5000, "still waiting for a position estimate");
            }
            else
            {
                RCLCPP_INFO_THROTTLE(logger, steady, 5000,
                                     "waiting for a fresh position estimate (last one %.1f s old)", odometry_age);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch (std::exception const &error)
    {
        RCLCPP_FATAL(logger, "waiting for a position estimate threw: %s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    if (stop_requested || !rclcpp::ok())
    {
        // Nothing has ticked, so nothing has moved and there is nothing to stop.
        RCLCPP_WARN(logger, "interrupted before the mission started");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(logger, "running mission '%s'", mission.c_str());
    auto status = BT::NodeStatus::RUNNING;
    // Wall-clock pacing so Ctrl-C is answered even while a sim is paused.
    // Everything inside the tree times itself with node->now(), so a paused
    // sim still pauses the mission.
    auto next_tick = std::chrono::steady_clock::now();
    try
    {
        while (!stop_requested && rclcpp::ok())
        {
            executor.spin_some();
            status = tree->tickOnce();
            if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
            {
                break;
            }
            // max(), not a plain +=: after a tick that overran 100 ms, catch up
            // to "now" instead of firing a burst of back-to-back ticks to make
            // up the lost time.
            next_tick = std::max(next_tick + std::chrono::milliseconds(100), std::chrono::steady_clock::now());
            std::this_thread::sleep_until(next_tick);
        }
    }
    catch (std::exception const &error)
    {
        RCLCPP_FATAL(logger, "mission '%s' threw: %s", mission.c_str(), error.what());
        status = BT::NodeStatus::FAILURE;
    }
    catch (...)
    {
        RCLCPP_FATAL(logger, "mission '%s' threw something that isn't a std::exception", mission.c_str());
        status = BT::NodeStatus::FAILURE;
    }

    if (stop_requested)
    {
        RCLCPP_WARN(logger, "interrupted; stopping the boat");
    }
    else
    {
        RCLCPP_INFO(logger, "mission '%s' finished: %s", mission.c_str(),
                    status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
    }
    // Each cleanup step runs even if the other one throws: whichever one
    // fails, the boat still gets the other's chance to stop it, and
    // rclcpp::shutdown() below still runs so the process never hangs on exit.
    try
    {
        tree->haltTree();
    }
    catch (std::exception const &error)
    {
        RCLCPP_ERROR(logger, "haltTree() threw: %s", error.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(logger, "haltTree() threw something that isn't a std::exception");
    }
    try
    {
        ctx->stop_motors();
    }
    catch (std::exception const &error)
    {
        RCLCPP_ERROR(logger, "stop_motors() threw: %s", error.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(logger, "stop_motors() threw something that isn't a std::exception");
    }

    // Keep saying zero for a moment before exiting. The empty plan goes to
    // guidance and the zero cmd_vel to thruster_manager -- different
    // processes -- so guidance can still emit one more drive command after our
    // zero has landed, and that command would stand until thruster_manager's
    // 1 s command_timeout. Repeating the zero every 100 ms for 0.5 s, longer
    // than guidance's 10 Hz period, gets the last word without waiting on that
    // timeout. A second Ctrl-C still kills the process at once (the handler
    // re-raises).
    try
    {
        auto const until = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < until)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            ctx->maneuvers->spinner.stop();
            executor.spin_some();
        }
    }
    catch (std::exception const &error)
    {
        RCLCPP_ERROR(logger, "repeating the stop threw: %s", error.what());
    }
    rclcpp::shutdown();
    return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
