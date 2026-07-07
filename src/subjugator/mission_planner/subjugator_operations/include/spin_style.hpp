#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <math.h>

#include <array>
#include <chrono>
#include <memory>

#include "context.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "subjugator_msgs/msg/thruster_efforts.hpp"

#include <tf2/tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/tf2/LinearMath/Quaternion.hpp>

// Momentum-spin about a body axis (roll or pitch). Both were identical
// three-phase state machines (disable controller -> wind up momentum in the
// reverse direction -> strongest command in the target direction -> stop,
// republish a yaw-only goal, re-enable controller). The ONLY axis-specific
// data is the sign of the four vertical thrusters {blv, brv, flv, frv} during
// the momentum wind-up; the strongest command is its exact negation, and the
// horizontal thrusters stay zero throughout. RollStyle/PitchStyle below are
// thin subclasses that supply that one vector, so both register and appear in
// XML under their original names.
class SpinStyle : public BT::ActionNodeBase
{
  public:
    SpinStyle(std::string const& name, const BT::NodeConfig& config, std::array<double, 4> momentum_vertical,
              char const* who)
      : BT::ActionNodeBase(name, config)
      , momentum_vertical_(momentum_vertical)
      , who_(who)
      , state_rn(BT::NodeStatus::IDLE)
    {
        get_port_data();
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<Context>>("ctx"),
            BT::InputPort<double>("milliseconds"),
            BT::InputPort<double>("momentum_ms"),
        };
    }

    void halt() override
    {
        state_rn = BT::NodeStatus::IDLE;
        gained_momentum = false;
    }

    BT::NodeStatus tick() override
    {
        // the action we take when ticked depends on our current status

        if (state_rn == BT::NodeStatus::IDLE)
        {
            // stop controller
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = false;  // false to disable the controller
            controller_request_result = ctx_->controller_enable_client->async_send_request(request).future.share();

            // setup clock to count seconds
            wait_duration_ = std::chrono::duration<double>(milliseconds_ / 1000.0);
            start_time_ = std::chrono::steady_clock::now();

            // move to next state
            state_rn = BT::NodeStatus::RUNNING;
            return state_rn;
        }

        if (state_rn == BT::NodeStatus::RUNNING && gained_momentum == false)
        {
            // wait for controller disable to complete
            if (controller_request_result.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
            {
                start_time_ = std::chrono::steady_clock::now();
                return BT::NodeStatus::RUNNING;
            }

            // spin in the OPPOSITE direction to build momentum
            auto msg = subjugator_msgs::msg::ThrusterEfforts();
            msg.thrust_blh = 0.0;
            msg.thrust_brh = 0.0;
            msg.thrust_flh = 0.0;
            msg.thrust_frh = 0.0;
            msg.thrust_blv = momentum_vertical_[0];
            msg.thrust_brv = momentum_vertical_[1];
            msg.thrust_flv = momentum_vertical_[2];
            msg.thrust_frv = momentum_vertical_[3];
            ctx_->raw_effort_pub->publish(msg);

            // check if the pre-spin momentum timer has elapsed
            auto now = std::chrono::steady_clock::now();
            if (now - start_time_ < momentum_duration_)
            {
                return BT::NodeStatus::RUNNING;
            }

            // momentum wind-up complete — stop the (vertical) thrusters we spun
            // up, flip flag. (PitchStyle historically zeroed the horizontal
            // thrusters here, which were already 0, so it never actually
            // stopped its momentum spin; zeroing the vertical set fixes that.)
            msg.thrust_blv = 0.0;
            msg.thrust_brv = 0.0;
            msg.thrust_flv = 0.0;
            msg.thrust_frv = 0.0;
            ctx_->raw_effort_pub->publish(msg);

            gained_momentum = true;

            // intentionally NOT re-enabling the controller here
            return BT::NodeStatus::RUNNING;
        }
        else if (state_rn == BT::NodeStatus::RUNNING && gained_momentum == true)
        {
            // gotta wait for the request to have actually sent lol
            // once the future completes, this will always be false so think of it as a guard
            // for the rest of the function until the request completes
            if (controller_request_result.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
            {
                start_time_ = std::chrono::steady_clock::now();
                return BT::NodeStatus::RUNNING;
            }

            // publish strongest spin command (exact negation of the momentum wind-up)
            auto msg = subjugator_msgs::msg::ThrusterEfforts();
            msg.thrust_blh = 0.0;  // duh
            msg.thrust_brh = 0.0;  // duh
            msg.thrust_flh = 0.0;  // duh
            msg.thrust_frh = 0.0;  // duh
            msg.thrust_blv = -momentum_vertical_[0];
            msg.thrust_brv = -momentum_vertical_[1];
            msg.thrust_flv = -momentum_vertical_[2];
            msg.thrust_frv = -momentum_vertical_[3];
            ctx_->raw_effort_pub->publish(msg);

            // how much time has passed?
            auto now = std::chrono::steady_clock::now();
            auto enough_time_passed = now - start_time_ >= wait_duration_;

            if (!enough_time_passed)
            {
                return BT::NodeStatus::RUNNING;
            }

            // if we made it here we are done:

            // stop the thrusters immediately
            msg.thrust_blh = 0.0;
            msg.thrust_brh = 0.0;
            msg.thrust_flh = 0.0;
            msg.thrust_frh = 0.0;
            msg.thrust_blv = 0.0;
            msg.thrust_brv = 0.0;
            msg.thrust_flv = 0.0;
            msg.thrust_frv = 0.0;
            ctx_->raw_effort_pub->publish(msg);

            // publish current pose as goal pose (so that the controller doesn't freak)
            auto our_pos_rn = ctx_->latest_odom;
            auto goal_msg = geometry_msgs::msg::Pose();

            goal_msg.position.x = our_pos_rn->pose.pose.position.x;
            goal_msg.position.y = our_pos_rn->pose.pose.position.y;
            goal_msg.position.z = our_pos_rn->pose.pose.position.z;

            // strip roll and pitch from the current pose, keep only yaw
            auto q_orig = our_pos_rn->pose.pose.orientation;
            auto q_quat = tf2::Quaternion(q_orig.x, q_orig.y, q_orig.z, q_orig.w);
            double roll = NAN, pitch = NAN, yaw = NAN;
            auto matrix = tf2::Matrix3x3(q_quat);
            matrix.getRPY(roll, pitch, yaw);
            tf2::Quaternion yaw_only;
            yaw_only.setRPY(0.0, 0.0, yaw);

            goal_msg.orientation.x = yaw_only.x();
            goal_msg.orientation.y = yaw_only.y();
            goal_msg.orientation.z = yaw_only.z();
            goal_msg.orientation.w = yaw_only.w();

            ctx_->last_goal_mx.lock();
            ctx_->goal_pub->publish(goal_msg);
            ctx_->last_goal_mx.unlock();

            // start controller (this will take a while so you should have a delay after this mission for the controller
            // to start back up :)
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = true;  // true to enable the controller
            controller_request_result = ctx_->controller_enable_client->async_send_request(request).future.share();

            // reset state and return success
            state_rn = BT::NodeStatus::IDLE;
            return BT::NodeStatus::SUCCESS;
        }

        else
        {  // THIS SHOULD BE UNREACHABLE
            return BT::NodeStatus::FAILURE;
        }
    }

  private:
    std::array<double, 4> momentum_vertical_;  // {blv, brv, flv, frv} signs during momentum wind-up
    char const* who_;                          // node name for port-error messages

    std::shared_ptr<Context> ctx_;
    BT::NodeStatus state_rn;  // IDLE = WAITING, RUNNING = mid spin, SUCCESS = spin completed!

    // stuff for keeping time think of it as a stop watch
    double milliseconds_;
    std::chrono::duration<double> wait_duration_;
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<long, std::ratio<1, 1000000000>>>
        start_time_;
    // ^ fuck you c++ this type is hideous

    bool gained_momentum = false;
    double momentum_ms_;
    std::chrono::duration<double> momentum_duration_;

    // stuff for checking if a request went through
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture controller_request_result;

    void get_port_data()
    {
        // ctx
        auto ctx_res = getInput<std::shared_ptr<Context>>("ctx");
        if (!ctx_res)
        {
            throw BT::RuntimeError(std::string(who_) + " requires [ctx] input");
        }
        ctx_ = ctx_res.value();

        // time in ms
        auto milliseconds_res = getInput<double>("milliseconds");
        if (!milliseconds_res)
        {
            throw BT::RuntimeError(std::string(who_) + " requires [milliseconds] input");
        }
        milliseconds_ = milliseconds_res.value();

        auto momentum_ms_res = getInput<double>("momentum_ms");
        if (!momentum_ms_res)
        {
            throw BT::RuntimeError(std::string(who_) + " requires [momentum_ms] input");
        }
        momentum_ms_ = momentum_ms_res.value();
        momentum_duration_ = std::chrono::duration<double>(momentum_ms_ / 1000.0);
    }

  protected:
};

// Roll about the body X axis: vertical thrusters alternate port/starboard.
class RollStyle : public SpinStyle
{
  public:
    RollStyle(std::string const& name, const BT::NodeConfig& config)
      : SpinStyle(name, config, { -1.0, 1.0, -1.0, 1.0 }, "RollStyle")
    {
    }
};

// Pitch about the body Y axis: vertical thrusters alternate fore/aft.
class PitchStyle : public SpinStyle
{
  public:
    PitchStyle(std::string const& name, const BT::NodeConfig& config)
      : SpinStyle(name, config, { -1.0, -1.0, 1.0, 1.0 }, "PitchStyle")
    {
    }
};
