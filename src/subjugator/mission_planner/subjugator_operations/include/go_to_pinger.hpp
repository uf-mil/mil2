#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/decorator_node.h>

#include <algorithm>
#include <cmath>

#include "context.hpp"
#include "mil_msgs/msg/processed_ping.hpp"
#include "publish_goal.hpp"

// just pass the pings to this and it'll tell you once we have passed the pinger
// handle all vector stuff, cmath stuff, stupid stuff
class PingChecker
{
  public:
    PingChecker() = default;

    // true if we passed the pinger, false if not
    bool new_ping(mil_msgs::msg::ProcessedPing const& new_ping)
    {
        // insert new ping into queue
        this->insert_new_ping(new_ping);

        // sanity check
        if (recent_pings_.size() <= 1)
        {
            return false;
        }

        // check for passing pinger
        bool passed_pinger = false;
        for (size_t i = 0; i < recent_pings_.size() - 1; i++)
        {  // ++i is stupid
            passed_pinger = compare_two_pings(recent_pings_[i], recent_pings_[i + 1]);
            if (passed_pinger == true)
            {
                break;
            }
        }

        // trim length
        trim_to_n_elements(10);

        // return result
        return passed_pinger;
    }

    // delete all recent pings
    void reset()
    {
        recent_pings_.clear();
    }

  private:
    std::deque<mil_msgs::msg::ProcessedPing> recent_pings_;

    void insert_new_ping(mil_msgs::msg::ProcessedPing const& new_ping)
    {
        recent_pings_.push_back(new_ping);
    }

    void trim_to_n_elements(size_t n)
    {
        for (; recent_pings_.size() > n; recent_pings_.pop_front())
        {
        }
    }

    // true if we passed the pinger, false if not
    bool compare_two_pings(mil_msgs::msg::ProcessedPing const& p1, mil_msgs::msg::ProcessedPing const& p2,
                           int max_degrees = 100)
    {
        auto x1 = p1.origin_direction_body.x;
        auto y1 = p1.origin_direction_body.y;
        auto x2 = p2.origin_direction_body.x;
        auto y2 = p2.origin_direction_body.y;

        // normalize them
        auto mag1 = std::sqrt(x1 * x1 + y1 * y1);
        auto mag2 = std::sqrt(x2 * x2 + y2 * y2);
        x1 /= mag1;  // TODO divide by 0 here?
        y1 /= mag1;
        x2 /= mag2;
        y2 /= mag2;

        // compute dot product
        auto dot12 = x1 * x2 + y1 * y2;
        dot12 = std::clamp(dot12, -1.0, 1.0);

        // get angle (it's acos of dot)
        auto angle_rad = std::acos(dot12);
        auto angle_degrees =
            (angle_rad * 180) / 3.14159265358979323846;  // would be better to use a named var not magic numbers idc

        return angle_degrees > max_degrees;
    }
};

class SonarFollower : public BT::DecoratorNode
{
  public:
    SonarFollower(std::string const& name, const BT::NodeConfig& config) : BT::DecoratorNode(name, config)
    {
        get_port_data();

        // sub to ping publisher
        sub_ = ctx_->node->create_subscription<mil_msgs::msg::ProcessedPing>(
            "hydrophones/solved", 10, [this](mil_msgs::msg::ProcessedPing const& msg) { this->topic_cb(msg); });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<Context>>("ctx"),

            BT::OutputPort<double>("sonar_x"),
            BT::OutputPort<double>("sonar_y"),
            BT::OutputPort<double>("sonar_z"),
            BT::OutputPort<double>("sonar_w"),
        };
    }

  private:
    std::shared_ptr<Context> ctx_;
    std::shared_ptr<rclcpp::Subscription<mil_msgs::msg::ProcessedPing>> sub_;
    BT::NodeStatus current_status_ = BT::NodeStatus::IDLE;

    // other nodes
    std::unique_ptr<PublishGoalPose> goal_pub_node_;

    // pinger stuff
    PingChecker pc;

    void get_port_data()
    {
        auto ctx_res = getInput<std::shared_ptr<Context>>("ctx");
        if (!ctx_res)
        {
            throw BT::RuntimeError("TopicTicker requires [ctx] input");
        }
        ctx_ = ctx_res.value();
    }

    // each ping we will move and check if done
    void topic_cb(mil_msgs::msg::ProcessedPing const& msg)
    {
        // check ping
        bool passed_the_pinger = pc.new_ping(msg);
        if (passed_the_pinger)
        {
            current_status_ = BT::NodeStatus::SUCCESS;
            return;
        }

        // move towards ping
        move_towards_ping(msg);
    }

    void move_towards_ping(mil_msgs::msg::ProcessedPing const& new_ping)
    {
        // this isn't dry :(
        auto x = new_ping.origin_direction_body.x;
        auto y = new_ping.origin_direction_body.y;
        auto mag = std::sqrt(x * x + y * y);
        x /= mag;
        y /= mag;
        // (x, y) dot (1, 0) = 1*1*cos(theta) (1 times 1 since both are unit length)
        // x = cos(theta)
        auto angle_rad = acos(std::clamp(x, -1.0, 1.0));
        auto z = std::sin(angle_rad / 2);
        auto w = std::cos(angle_rad / 2);

        // set outputs
        setOutput("sonar_x", x);
        setOutput("sonar_y", y);
        setOutput("sonar_z", z);
        setOutput("sonar_w", w);

        // Tick the child
        child_node_->executeTick();
    }

  protected:
    BT::NodeStatus tick() override
    {
        if (current_status_ == BT::NodeStatus::IDLE)
        {
            current_status_ = BT::NodeStatus::RUNNING;
        }

        auto temp = current_status_;
        if (current_status_ == BT::NodeStatus::SUCCESS)
        {
            current_status_ = BT::NodeStatus::IDLE;
        }

        return temp;
    }

    // TODO halt
    // custom halt needed since node has internal state
    void halt() override
    {
        BT::DecoratorNode::halt();
        current_status_ = BT::NodeStatus::IDLE;
        pc.reset();
    }
};
