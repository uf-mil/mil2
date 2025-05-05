#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mil_msgs/action/bag_online.hpp"
#include "mil_msgs/srv/bag_topics.hpp"
#include "online_bagger/common.hpp"
#include "online_bagger/client.hpp"

#include <atomic>

namespace online_bagger
{

Client::Client():rclcpp::Node("online_bagger_client")
{
    action_client = rclcpp_action::create_client<mil_msgs::action::BagOnline>(this, BAG_ACTION_NAME);
    srv_client = create_client<mil_msgs::srv::BagTopics>(TOPIC_SERVICE_NAME);

    if(action_client->wait_for_action_server(std::chrono::seconds(1)) == true)
        state = State::Ready;

    alive_timer = create_wall_timer(std::chrono::milliseconds(100), std::bind(&Client::is_alive, this));
}

Client::~Client()
{

}

Client::State Client::get_state()
{
    return state;
}

    
std::shared_future<std::shared_ptr<mil_msgs::srv::BagTopics_Response>> 
    Client::get_bag_topics(std::function<void(rclcpp::Client<mil_msgs::srv::BagTopics>::SharedFuture future)> callback)
{
    auto request = std::make_shared<mil_msgs::srv::BagTopics::Request>();
    return srv_client->async_send_request(request, callback).future;
}

std::shared_future<Client::BagOnlineGoalHandle::SharedPtr>
    Client::start_bagging(const mil_msgs::action::BagOnline::Goal& goal,
    const rclcpp_action::Client<mil_msgs::action::BagOnline>::SendGoalOptions& options)
{
    state = State::Bagging;
    return action_client->async_send_goal(goal, options);
}

void Client::finish_bagging()
{
    action_client->async_cancel_all_goals();
    state = State::Ready;
}

void Client::is_alive()
{
    if(state == State::Waiting && action_client->action_server_is_ready())
    {
        state = State::Ready;
    }
    else if(state == State::Ready && !action_client->action_server_is_ready())
    {
        state = State::Waiting;
    }
    else if(state == State::Bagging && !action_client->action_server_is_ready())
    {
        state = State::Waiting;
    }
}

}
