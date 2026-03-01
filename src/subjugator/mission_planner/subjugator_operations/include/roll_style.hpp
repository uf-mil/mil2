#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <math.h>

#include <chrono>
#include <memory>

#include "context.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "subjugator_msgs/msg/thruster_efforts.hpp"

#include <tf2/tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/tf2/LinearMath/Quaternion.hpp>

class RollStyle : public BT::SyncActionNode
{
  public:
    RollStyle(std::string const& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config), state_rn(BT::NodeStatus::IDLE)
    {
        get_port_data();
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<Context>>("ctx"),
            BT::InputPort<double>("milliseconds"),
        };
    }

  private:
    std::shared_ptr<Context> ctx_;
    BT::NodeStatus state_rn;  // IDLE = WAITING, RUNNING = mid spin, SUCCESS = spin completed!

    // stuff for keeping time think of it as a stop watch
    double milliseconds_;
    std::chrono::duration<double> wait_duration_;
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<long, std::ratio<1, 1000000000>>>
        start_time_;
    // ^ fuck you c++ this type is hideous

    // stuff for checking if a request went through
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture controller_request_result;

    void get_port_data()
    {
        // ctx
        auto ctx_res = getInput<std::shared_ptr<Context>>("ctx");
        if (!ctx_res)
        {
            throw BT::RuntimeError("RollStyle requires [ctx] input");
        }
        ctx_ = ctx_res.value();

        // time in ms
        auto milliseconds_res = getInput<double>("milliseconds");
        if (!milliseconds_res)
        {
            throw BT::RuntimeError("RollStyle requires [milliseconds] input");
        }
        milliseconds_ = milliseconds_res.value();
    }

  protected:
    BT::NodeStatus tick() override
    {
        // the action we take when ticked depends on our current status

        if (state_rn == BT::NodeStatus::IDLE)
        {
            // record localization's position (maybe)

            // stop controller
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = false;  // false to disable the controller
            controller_request_result = ctx_->controller_enable_client->async_send_request(request).future.share();

            // setup clock to count seconds
            wait_duration_ = std::chrono::duration<double>(milliseconds_ * 1000);
            start_time_ = std::chrono::steady_clock::now();

            // move to next state
            state_rn = BT::NodeStatus::RUNNING;
            return state_rn;
        }

        else if (state_rn == BT::NodeStatus::RUNNING)
        {
            // gotta wait for the request to have actually sent lol
            // once the future completes, this will always be false so think of it as a guard
            // for the rest of the function until the request completes
            if (controller_request_result.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
            {
                start_time_ = std::chrono::steady_clock::now();
                return BT::NodeStatus::RUNNING;
            }

            // publish strongest roll command
            auto msg = subjugator_msgs::msg::ThrusterEfforts();
            msg.thrust_blh = 0.0;  // duh
            msg.thrust_brh = 0.0;  // duh
            msg.thrust_flh = 0.0;  // duh
            msg.thrust_frh = 0.0;  // duh
            msg.thrust_blv = 1.0;
            msg.thrust_brv = -1.0;
            msg.thrust_flv = 1.0;
            msg.thrust_frv = -1.0;
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

            // horrific Quaternion math to safely remove roll and pitch from current pose
            // if your reading this and think of the equation z^2 = sqrt(1-w^2) I will personally slaughter you
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
};
