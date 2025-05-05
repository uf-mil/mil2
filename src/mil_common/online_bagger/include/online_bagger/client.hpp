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

    enum class State
    {
        Waiting,
        Ready,
        Bagging
    };

    Client();
    ~Client();

    State get_state();
    
    std::shared_future<std::shared_ptr<mil_msgs::srv::BagTopics_Response>> 
        get_bag_topics(std::function<void(rclcpp::Client<mil_msgs::srv::BagTopics>::SharedFuture future)> callback);

    std::shared_future<BagOnlineGoalHandle::SharedPtr>
        start_bagging(const mil_msgs::action::BagOnline::Goal& goal,
        const rclcpp_action::Client<mil_msgs::action::BagOnline>::SendGoalOptions& options);

    void finish_bagging();

    private:

    std::atomic<State> state;
    rclcpp_action::Client<mil_msgs::action::BagOnline>::SharedPtr action_client;
    rclcpp::Client<mil_msgs::srv::BagTopics>::SharedPtr srv_client;
    rclcpp::TimerBase::SharedPtr alive_timer;

    void is_alive();
    
};

}

