#include "start_coin_flip.hpp"

#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <thread>

#include <ament_index_cpp/get_package_prefix.hpp>

extern char** environ;

namespace
{
// Installed path of the classifier script: <prefix>/lib/subjugator_vision/coin_flip_node.py
std::string coinFlipScriptPath()
{
    return ament_index_cpp::get_package_prefix("subjugator_vision") + "/lib/subjugator_vision/coin_flip_node.py";
}
}  // namespace

void stopCoinFlip(Context& ctx)
{
    std::scoped_lock lk(ctx.child_mx);
    if (ctx.coin_flip_pid <= 0)
        return;

    RCLCPP_INFO(ctx.logger(), "Stopping coin_flip_node (pid=%d)", ctx.coin_flip_pid);
    kill(ctx.coin_flip_pid, SIGINT);  // ask rclpy to shut down cleanly

    // Wait up to ~3s for it to exit, then force it.
    for (int i = 0; i < 30; ++i)
    {
        if (waitpid(ctx.coin_flip_pid, nullptr, WNOHANG) != 0)
        {
            ctx.coin_flip_pid = -1;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    kill(ctx.coin_flip_pid, SIGKILL);
    waitpid(ctx.coin_flip_pid, nullptr, 0);
    ctx.coin_flip_pid = -1;
}

StartCoinFlip::StartCoinFlip(std::string const& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList StartCoinFlip::providedPorts()
{
    return { BT::InputPort<double>("timeout_s", 30.0, "Seconds to wait for the node to start publishing"),
             BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context") };
}

BT::NodeStatus StartCoinFlip::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;
    (void)getInput("timeout_s", timeout_s_);

    {
        std::scoped_lock lk(ctx_->child_mx);
        if (ctx_->coin_flip_pid > 0)
        {
            RCLCPP_INFO(ctx_->logger(), "StartCoinFlip: coin_flip_node already running (pid=%d)", ctx_->coin_flip_pid);
        }
        else if (!launchNode())
        {
            return BT::NodeStatus::FAILURE;
        }
    }

    // Clear any stale class so we wait for *fresh* data from this run.
    {
        std::scoped_lock lk(ctx_->wall_direction_mx);
        ctx_->latest_wall_direction.reset();
    }
    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
}

// Spawns "python3 <coin_flip_node.py>" as a child process and records its PID.
// Caller must hold ctx_->child_mx. Returns false on failure.
bool StartCoinFlip::launchNode()
{
    std::string const script = coinFlipScriptPath();
    if (!std::filesystem::exists(script))
    {
        RCLCPP_ERROR(ctx_->logger(), "StartCoinFlip: script not found at %s (rebuild subjugator_vision?)",
                     script.c_str());
        return false;
    }

    // posix_spawnp searches PATH for python3 and inherits our (ROS-sourced)
    // environment, so rclpy and the package are already on PYTHONPATH.
    char* argv[] = { const_cast<char*>("python3"), const_cast<char*>(script.c_str()), nullptr };
    pid_t pid = 0;
    int const rc = posix_spawnp(&pid, "python3", nullptr, nullptr, argv, environ);
    if (rc != 0)
    {
        RCLCPP_ERROR(ctx_->logger(), "StartCoinFlip: posix_spawnp(python3) failed (rc=%d)", rc);
        return false;
    }

    ctx_->coin_flip_pid = pid;
    RCLCPP_INFO(ctx_->logger(), "StartCoinFlip: launched coin_flip_node (pid=%d)", pid);
    return true;
}

BT::NodeStatus StartCoinFlip::onRunning()
{
    bool up = false;
    {
        std::scoped_lock lk(ctx_->wall_direction_mx);
        up = ctx_->latest_wall_direction.has_value();
    }
    if (up)
    {
        RCLCPP_INFO(ctx_->logger(), "StartCoinFlip: coin_flip_node is publishing on /coin_flip/direction");
        return BT::NodeStatus::SUCCESS;
    }

    double const elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed > timeout_s_)
    {
        RCLCPP_ERROR(ctx_->logger(), "StartCoinFlip: timed out after %.1fs waiting for /coin_flip/direction",
                     timeout_s_);
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void StartCoinFlip::onHalted()
{
}
