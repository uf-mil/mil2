#include "start_coin_flip.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <thread>

void stopCoinFlip(Context& ctx)
{
    std::scoped_lock lk(ctx.child_mx);
    if (ctx.coin_flip_pid <= 0)
        return;

    RCLCPP_INFO(ctx.logger(), "Stopping coin_flip_node (pid=%d)", ctx.coin_flip_pid);
    killpg(ctx.coin_flip_pid, SIGINT);

    bool reaped = false;
    for (int i = 0; i < 30; ++i)  // up to ~3s for a graceful shutdown
    {
        if (waitpid(ctx.coin_flip_pid, nullptr, WNOHANG) != 0)
        {
            reaped = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!reaped)
    {
        killpg(ctx.coin_flip_pid, SIGKILL);
        waitpid(ctx.coin_flip_pid, nullptr, 0);
    }
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
            RCLCPP_INFO(ctx_->logger(), "StartCoinFlip: coin_flip_node already launched (pid=%d)", ctx_->coin_flip_pid);
        }
        else
        {
            pid_t pid = fork();
            if (pid < 0)
            {
                RCLCPP_ERROR(ctx_->logger(), "StartCoinFlip: fork() failed");
                return BT::NodeStatus::FAILURE;
            }
            if (pid == 0)
            {
                // Child: become its own process group leader so a terminal SIGINT
                // does not reach it; the mission planner kills it explicitly.
                setpgid(0, 0);
                execlp("ros2", "ros2", "run", "subjugator_vision", "coin_flip_node.py", static_cast<char*>(nullptr));
                _exit(127);  // execlp only returns on failure
            }
            ctx_->coin_flip_pid = pid;
            RCLCPP_INFO(ctx_->logger(), "StartCoinFlip: launched coin_flip_node (pid=%d)", pid);
        }
    }

    // Wait for *fresh* data so we know the node is really up this run.
    {
        std::scoped_lock lk(ctx_->wall_direction_mx);
        ctx_->latest_wall_direction.reset();
    }
    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
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
