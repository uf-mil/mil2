#include "actuate_servo.hpp"

#include <chrono>
#include <future>

#include <rclcpp/rclcpp.hpp>

namespace
{
// Servo.angle is interpreted as PWM duty cycle x10 by servo_controller/driver.py
// (duty = angle / 10, fed to RPi.GPIO ChangeDutyCycle which accepts 0-100). Reject
// anything that would command an out-of-range duty cycle. See Servo.srv for the contract.
constexpr int MAX_SERVO_ANGLE = 1000;  // duty 100.0, the RPi.GPIO upper bound
}  // namespace

ActuateServo::ActuateServo(std::string const& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList ActuateServo::providedPorts()
{
    return { BT::InputPort<std::string>("target", "Servo to actuate: dropper | gripper | torpedo"),
             BT::InputPort<int>("angle", 0, "Angle value passed to the Servo service (PWM duty x10, 0-1000)"),
             BT::InputPort<int>("service_timeout_msec", 3000,
                                "Max time to wait for the Servo service to appear before failing"),
             BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context") };
}

BT::NodeStatus ActuateServo::onStart()
{
    if (!require_ctx(*this, ctx_, "ActuateServo"))
    {
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("target", target_) || target_.empty())
    {
        RCLCPP_ERROR(ctx_->logger(), "ActuateServo: missing 'target' (dropper | gripper | torpedo)");
        return BT::NodeStatus::FAILURE;
    }

    // The 'angle' port has a default (0), so an omitted port resolves successfully.
    // A false return means the port WAS bound but could not be resolved (e.g. an
    // unset blackboard key) -- fail loudly instead of silently defaulting to 0=CLOSE,
    // which would e.g. shut the gripper during the "open before descent" step.
    int angle_in = 0;
    if (!getInput("angle", angle_in))
    {
        RCLCPP_ERROR(ctx_->logger(), "ActuateServo: 'angle' port is set but could not be resolved");
        return BT::NodeStatus::FAILURE;
    }
    if (angle_in < 0 || angle_in > MAX_SERVO_ANGLE)
    {
        RCLCPP_ERROR(ctx_->logger(), "ActuateServo: angle %d out of range [0, %d]", angle_in, MAX_SERVO_ANGLE);
        return BT::NodeStatus::FAILURE;
    }
    angle_ = static_cast<uint16_t>(angle_in);
    (void)getInput("service_timeout_msec", service_timeout_msec_);

    if (target_ == "dropper")
    {
        client_ = ctx_->dropper_client;
    }
    else if (target_ == "gripper")
    {
        client_ = ctx_->gripper_client;
    }
    else if (target_ == "torpedo")
    {
        client_ = ctx_->torpedo_client;
    }
    else
    {
        RCLCPP_ERROR(ctx_->logger(), "ActuateServo: unknown target '%s'", target_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (!client_)
    {
        RCLCPP_ERROR(ctx_->logger(), "ActuateServo: client for '%s' not initialized", target_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (client_->service_is_ready())
    {
        return sendRequest();
    }

    // Service not yet discovered -- common right after bringup before the gripper
    // plugin / servo_controller advertises. Wait for it instead of failing the grasp
    // cycle (and burning a bounded retry) on a service that is merely slow to appear.
    RCLCPP_WARN(ctx_->logger(), "ActuateServo: service '%s' not ready; waiting up to %d ms", target_.c_str(),
                service_timeout_msec_);
    awaiting_service_ = true;
    deadline_ = ctx_->node->now() + rclcpp::Duration::from_seconds(service_timeout_msec_ / 1000.0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActuateServo::sendRequest()
{
    auto request = std::make_shared<Servo::Request>();
    request->angle = angle_;

    // async_send_request returns a FutureAndRequestId holding a std::future; share() it
    // so we can store and poll it across multiple onRunning ticks.
    auto fid = client_->async_send_request(request);
    future_ = fid.future.share();
    in_flight_ = true;
    awaiting_service_ = false;

    RCLCPP_INFO(ctx_->logger(), "ActuateServo: sent %s angle=%u", target_.c_str(),
                static_cast<unsigned>(request->angle));

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActuateServo::onRunning()
{
    if (awaiting_service_)
    {
        if (client_ && client_->service_is_ready())
        {
            return sendRequest();
        }
        if (ctx_->node->now() >= deadline_)
        {
            RCLCPP_ERROR(ctx_->logger(), "ActuateServo: service '%s' still not ready after %d ms", target_.c_str(),
                         service_timeout_msec_);
            awaiting_service_ = false;
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    if (!in_flight_)
    {
        return BT::NodeStatus::FAILURE;
    }

    auto status = future_.wait_for(std::chrono::milliseconds(0));
    if (status != std::future_status::ready)
    {
        return BT::NodeStatus::RUNNING;
    }

    // Response is empty; arrival means the server returned. get() can rethrow if the
    // server died or the call was interrupted -- catch it so the grasp fails the leaf
    // gracefully instead of unwinding tickOnce() and aborting the mission planner.
    in_flight_ = false;
    try
    {
        (void)future_.get();
    }
    catch (std::exception const& e)
    {
        RCLCPP_ERROR(ctx_->logger(), "ActuateServo: %s service call failed: %s", target_.c_str(), e.what());
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(ctx_->logger(), "ActuateServo: %s done", target_.c_str());
    return BT::NodeStatus::SUCCESS;
}

void ActuateServo::onHalted()
{
    in_flight_ = false;
    awaiting_service_ = false;
}
