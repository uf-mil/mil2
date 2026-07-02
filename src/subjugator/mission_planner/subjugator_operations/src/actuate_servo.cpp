#include "actuate_servo.hpp"

#include <chrono>
#include <future>

#include <rclcpp/rclcpp.hpp>

REGISTER(ActuateServo)

ActuateServo::ActuateServo(std::string const& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList ActuateServo::providedPorts()
{
    return { BT::InputPort<std::string>("target", "Servo to actuate: dropper | gripper | torpedo"),
             BT::InputPort<int>("angle", 0, "Angle value passed to the Servo service (uint16)"),
             BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context") };
}

BT::NodeStatus ActuateServo::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "ActuateServo: missing ctx on blackboard");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("target", target_) || target_.empty())
    {
        RCLCPP_ERROR(ctx_->logger(), "ActuateServo: missing 'target' (dropper | gripper | torpedo)");
        return BT::NodeStatus::FAILURE;
    }

    int angle_in = 0;
    (void)getInput("angle", angle_in);
    if (angle_in < 0 || angle_in > 0xFFFF)
    {
        RCLCPP_ERROR(ctx_->logger(), "ActuateServo: angle %d out of uint16 range", angle_in);
        return BT::NodeStatus::FAILURE;
    }

    rclcpp::Client<Servo>::SharedPtr client;
    if (target_ == "dropper")
    {
        client = ctx_->dropper_client;
    }
    else if (target_ == "gripper")
    {
        client = ctx_->gripper_client;
    }
    else if (target_ == "torpedo")
    {
        client = ctx_->torpedo_client;
    }
    else
    {
        RCLCPP_ERROR(ctx_->logger(), "ActuateServo: unknown target '%s'", target_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (!client)
    {
        RCLCPP_ERROR(ctx_->logger(), "ActuateServo: client for '%s' not initialized", target_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (!client->service_is_ready())
    {
        RCLCPP_WARN(ctx_->logger(), "ActuateServo: service '%s' not ready", target_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<Servo::Request>();
    request->angle = static_cast<uint16_t>(angle_in);

    // async_send_request returns a FutureAndRequestId holding a std::future; share() it
    // so we can store and poll it across multiple onRunning ticks.
    auto fid = client->async_send_request(request);
    future_ = fid.future.share();
    in_flight_ = true;

    RCLCPP_INFO(ctx_->logger(), "ActuateServo: sent %s angle=%u", target_.c_str(),
                static_cast<unsigned>(request->angle));

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActuateServo::onRunning()
{
    if (!in_flight_)
    {
        return BT::NodeStatus::FAILURE;
    }

    auto status = future_.wait_for(std::chrono::milliseconds(0));
    if (status != std::future_status::ready)
    {
        return BT::NodeStatus::RUNNING;
    }

    // Response is empty; arrival means the server returned.
    (void)future_.get();
    in_flight_ = false;

    RCLCPP_INFO(ctx_->logger(), "ActuateServo: %s done", target_.c_str());
    return BT::NodeStatus::SUCCESS;
}

void ActuateServo::onHalted()
{
    in_flight_ = false;
}
