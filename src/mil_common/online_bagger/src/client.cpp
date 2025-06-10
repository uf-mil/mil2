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

    action_client->wait_for_action_server(std::chrono::milliseconds(500));
}

Client::~Client()
{

}

Client::State Client::get_state()
{
    if(!action_client->action_server_is_ready())
        return State::Waiting;
    
    if(!bagging)
        return State::Ready;

    return State::Bagging;
}
    
Client::TopicsFuture Client::get_bag_topics(std::function<void(Client::TopicsFuture)> callback)
{
    std::shared_ptr<TopicsPromise> promise = std::make_shared<TopicsPromise>();
    TopicsFuture future = promise->get_future().share();

    auto request = std::make_shared<mil_msgs::srv::BagTopics::Request>();
    srv_client->async_send_request(request, [promise, future, callback](rclcpp::Client<mil_msgs::srv::BagTopics>::SharedFuture response_future) {
        promise->set_value(std::move(response_future.get()->topics));
        if(callback)
            callback(future);
    });

    return future;
}

Client::BagFuture Client::bag(const Client::BagOptions& options)
{
    std::shared_ptr<BagPromise> promise = std::make_shared<BagPromise>();
    BagFuture future = promise->get_future().share();

    bagging = true;

    rclcpp_action::Client<mil_msgs::action::BagOnline>::SendGoalOptions goal_options;
    goal_options.goal_response_callback = [this, on_finish = options.on_finish, promise, future](Client::BagOnlineGoalHandle::SharedPtr goal_handle){
        if(!goal_handle)
        {
            promise->set_value({false, "Bag request is rejected by the online bagger server."});
            if(on_finish)
                on_finish(future);
            bagging = false;
        }
    };

    if(options.on_progress)
    {
        goal_options.feedback_callback = [on_progress = options.on_progress]([[maybe_unused]]Client::BagOnlineGoalHandle::SharedPtr goal_handle, 
            const std::shared_ptr<const mil_msgs::action::BagOnline::Feedback> feedback){
            on_progress(feedback->progress);
        };
    }

    goal_options.result_callback = [this, on_finish = options.on_finish, promise, future](const Client::BagOnlineGoalHandle::WrappedResult& result){
        promise->set_value({result.result->success, std::move(result.result->status)});
        if(on_finish)
            on_finish(future);
        bagging = false;
    };

    action_client->async_send_goal(options.goal, goal_options);

    return future;
}

}
