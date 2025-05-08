#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client.hpp>

#include "mil_msgs/srv/bag_topics.hpp"
#include "mil_msgs/action/bag_online.hpp"

namespace online_bagger
{
class Client: public rclcpp::Node
{
    public:
    using BagOnlineGoalHandle = rclcpp_action::ClientGoalHandle<mil_msgs::action::BagOnline>;
    using TopicsFuture = std::shared_future<std::vector<std::string>>;
    using BagFuture = std::shared_future<std::pair<bool, std::string>>;

    enum class State
    {
        Waiting,
        Ready,
        Bagging
    };

    struct BagOptions
    {
        BagOptions()
        {
            goal.bag_time = 1.0f;
        }

        mil_msgs::action::BagOnline::Goal goal;
        std::function<void(float)> on_progress = nullptr;
        std::function<void(BagFuture)> on_finish = nullptr;
    };

    Client();
    ~Client();
    
    State get_state();
    
    TopicsFuture get_bag_topics(std::function<void(TopicsFuture)> callback = nullptr);
    BagFuture bag(const BagOptions& options);

    private:
    using TopicsPromise = std::promise<std::vector<std::string>>;
    using BagPromise = std::promise<std::pair<bool,std::string>>;

    std::atomic<bool> bagging;
    rclcpp_action::Client<mil_msgs::action::BagOnline>::SharedPtr action_client;
    rclcpp::Client<mil_msgs::srv::BagTopics>::SharedPtr srv_client;
    rclcpp::TimerBase::SharedPtr alive_timer;

    std::function<void(State old_state, State new_state)> on_state_change;
    
};

}

