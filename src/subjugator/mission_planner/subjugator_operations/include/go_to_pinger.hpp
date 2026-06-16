#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/decorator_node.h>

#include <deque>

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
    bool new_ping(mil_msgs::msg::ProcessedPing const& new_ping);

    // delete all recent pings
    void reset();

  private:
    std::deque<mil_msgs::msg::ProcessedPing> recent_pings_;

    void insert_new_ping(mil_msgs::msg::ProcessedPing const& new_ping);

    void trim_to_n_elements(size_t n);

    // true if we passed the pinger, false if not
    bool compare_two_pings(mil_msgs::msg::ProcessedPing const& p1, mil_msgs::msg::ProcessedPing const& p2,
                           int max_degrees = 100);
};

class SonarFollower : public BT::DecoratorNode
{
  public:
    SonarFollower(std::string const& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

  private:
    std::shared_ptr<Context> ctx_;
    std::shared_ptr<rclcpp::Subscription<mil_msgs::msg::ProcessedPing>> sub_;
    BT::NodeStatus current_status_ = BT::NodeStatus::IDLE;
    bool stop_on_first_ping_ = false;
    uint32_t target_freq_ = 0;
    uint32_t target_freq_tol_ = 0;

    // other nodes
    std::unique_ptr<PublishGoalPose> goal_pub_node_;

    // pinger stuff
    PingChecker pc;

    void get_port_data();

    // each ping we will move and check if done
    void topic_cb(mil_msgs::msg::ProcessedPing const& msg);

    void move_towards_ping(mil_msgs::msg::ProcessedPing const& new_ping);

  protected:
    BT::NodeStatus tick() override;

    // TODO halt
    // custom halt needed since node has internal state
    void halt() override;
};
