#include <rclcpp_action/rclcpp_action.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mil_msgs/action/bag_online.hpp"
#include "mil_msgs/srv/bag_topics.hpp"
#include "online_bagger/online_bagger.h"

#include <boost/process/environment.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <thread>
#include <chrono>
#include <filesystem>
#include <atomic>

namespace online_bagger
{
class Server: public rclcpp::Node
{
    public:
    
    Server():rclcpp::Node("online_bagger_server")
    {
        get_params();

        topics_service = create_service<mil_msgs::srv::BagTopics>(TOPIC_SERVICE_NAME, 
            std::bind(&Server::handle_topics_req, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        _action_server = rclcpp_action::create_server<mil_msgs::action::BagOnline>(this,
            BAG_ACTION_NAME,
            std::bind(&Server::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Server::handle_cancel, this, std::placeholders::_1),
            std::bind(&Server::handle_accepted, this, std::placeholders::_1)
        );
        
        subscribe();
        if(subscriber_list.size() == successful_subscription_count && subscriber_list.size() != 0)
        {
            RCLCPP_INFO(get_logger(), "Subscribed to %ld of %ld topics", 
                                successful_subscription_count,
                                subscriber_list.size());
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Subscribed to %ld of %ld topics, will try again every %.2f seconds", 
                                successful_subscription_count,
                                subscriber_list.size(),
                                resubscribe_period);
            resubscriber = create_wall_timer(std::chrono::milliseconds(static_cast<int>(resubscribe_period*1000)), [this]{
                if ((successful_subscription_count == subscriber_list.size()) && resubscriber)
                {
                    resubscriber->cancel();
                    if(subscriber_list.size() == 0)
                    {
                        RCLCPP_WARN(get_logger(), "No topics selected to subscribe to. Closing.");
                        rclcpp::shutdown();
                    }
                    else
                    {
                        RCLCPP_INFO(get_logger(), "All topics are subscribed! Shutting down resubscriber");
                    }
                    
                    return;
                }
                subscribe();
            });
        }
    }
    ~Server()
    {

    }

    private:
    using BagOnlineGoalHandle = rclcpp_action::ServerGoalHandle<mil_msgs::action::BagOnline>;
    std::atomic_bool streaming = true;
    rclcpp_action::Server<mil_msgs::action::BagOnline>::SharedPtr _action_server;
    rclcpp::Service<mil_msgs::srv::BagTopics>::SharedPtr topics_service;
    size_t successful_subscription_count = 0;
    size_t stream_time;
    bool dated_folder;
    float resubscribe_period;
    std::filesystem::path dir;
    std::unordered_map<std::string, std::pair<size_t, rclcpp::GenericSubscription::SharedPtr>> subscriber_list;
    std::unordered_map<std::string, std::deque<std::pair<rclcpp::Time, std::shared_ptr<rclcpp::SerializedMessage>>>> topic_messages;
    rclcpp::TimerBase::SharedPtr resubscriber;
    size_t iteration_count = 0;

    void handle_topics_req([[maybe_unused]]mil_msgs::srv::BagTopics::Request::ConstSharedPtr request,
        mil_msgs::srv::BagTopics::Response::SharedPtr response)
    {
        for(const auto& [topic, info] : subscriber_list)
        {
            const auto& [time, subscribed] = info;
            if(subscribed != nullptr)
                response->topics.push_back(topic);
        }
    }

    rclcpp_action::GoalResponse handle_goal(
        [[maybe_unused]]const rclcpp_action::GoalUUID & uuid, 
        [[maybe_unused]]std::shared_ptr<const mil_msgs::action::BagOnline::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Accepted goal from online bagger client");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        [[maybe_unused]]const std::shared_ptr<BagOnlineGoalHandle> goal_handle)
    {
        return rclcpp_action::CancelResponse::REJECT;
    }

    float get_topic_duration(const std::string& topic)
    {
        if(topic_messages[topic].size() == 0)
            return 0.0f;
        
        return (topic_messages[topic].back().first - topic_messages[topic].front().first).nanoseconds() / 1e-9f;
    }

    size_t get_time_index(const std::string& topic, float requested_seconds)
    {
        if(requested_seconds == 0)
            return 0;
        
        float topic_duration = get_topic_duration(topic);

        if(topic_duration == 0)
            return 0;
        
        float ratio = requested_seconds / topic_duration;
        size_t index = topic_messages[topic].size() * (1 - std::min(ratio, 1.0f));
        return index;
    }

    void handle_accepted(std::shared_ptr<BagOnlineGoalHandle> goal_handle)
    {
        std::thread(std::bind(&Server::start_bagging, this, std::placeholders::_1), goal_handle).detach();
    }

    void bagger_callback(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string& topic)
    {
        if(!streaming)
            return;
        
        
        iteration_count += 1;
        topic_messages[topic].push_back({now(), msg});

        float time_diff = get_topic_duration(topic);

        if(iteration_count % 100 == 0)
        {
            RCLCPP_DEBUG(get_logger(), "%s has %ld messages spaning %.2f seconds.", 
                                        topic.c_str(), 
                                        topic_messages[topic].size(),
                                        time_diff);
        }

        while(time_diff > subscriber_list[topic].first && rclcpp::ok())
        {
            topic_messages[topic].pop_front();
            time_diff = get_topic_duration(topic);
        }

    }

    std::filesystem::path get_bag_name(const std::filesystem::path& filename)
    {
        std::filesystem::path default_dir = dir;
        std::string date = boost::gregorian::to_simple_string(boost::gregorian::day_clock::local_day());
        std::string time = boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time().time_of_day());
        if(dated_folder)
        {
            default_dir /= date;
        }

        std::filesystem::path bag_dir = default_dir / filename.parent_path();
        std::filesystem::path bag_name = filename.filename();
        if(!std::filesystem::exists(bag_dir))
        {
            std::filesystem::create_directories(bag_dir);
        }

        if(!bag_name.has_filename())
        {
            bag_name.replace_filename(date + '-' + time);
        }

        if(bag_name.extension() != "bag")
        {
            bag_name.replace_extension("bag");
        }

        return bag_dir / bag_name;
    }

    void start_bagging(std::shared_ptr<BagOnlineGoalHandle> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Start bagging");
        auto result = std::make_shared<mil_msgs::action::BagOnline::Result>();
        if(!streaming.exchange(false))
        {
            result->status = "Bag Request came in while bagging, priority given to prior request",
            result->success = false;
            goto ret;
        }

        try
        {
            const auto goal = goal_handle->get_goal();
            std::filesystem::path filename = get_bag_name(goal->bag_name).string();

            float requested_seconds = goal->bag_time;
            const std::vector<std::string>& selected_topics = goal->topics;

            auto feedback = std::make_shared<mil_msgs::action::BagOnline::Feedback>();
            size_t total_messages = 0;
            std::unordered_map<std::string, size_t> bag_topics;
            for(const auto& topic : selected_topics)
            {
                auto it = subscriber_list.find(topic);
                if(it == subscriber_list.end())
                    continue;
                
                const auto& [time, subscribed] = it->second;
                if(subscribed == nullptr)
                    continue;

                if(topic_messages[topic].size() == 0)
                    continue;

                size_t index = get_time_index(topic, requested_seconds);
                total_messages += topic_messages[topic].size() - index;
                bag_topics[topic] = index;
            }

            if(total_messages == 0)
            {
                result->success = false;
                result->status = "No messages to bag";
                goto ret;
            }

            auto bag = std::make_shared<rosbag2_cpp::Writer>();
            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = filename.string();
            bag->open(storage_options);

            goal_handle->publish_feedback(feedback);

            auto topic_names_and_types = get_topic_names_and_types();

            size_t msg_inc = 0;
            for(const auto& [topic, index] : bag_topics)
            {
                for(auto it=topic_messages[topic].begin(); 
                         it!=topic_messages[topic].end();
                         it++)
                {
                    bag->write(it->second, topic, topic_names_and_types[topic][0],it->first);
                    if(msg_inc % 50 == 0)  // send feedback every 50 messages
                    {
                        feedback->progress = static_cast<float>(msg_inc) / total_messages;
                        goal_handle->publish_feedback(feedback);
                    }
                    msg_inc += 1;
                    // empty deque when done writing to bag
                    topic_messages[topic].clear();
                }
            }
            feedback->progress = 1.0;
            goal_handle->publish_feedback(feedback);
            bag->close();

            result->success = true;
        }
        catch(const std::exception& e)
        {
            result->status = e.what();
            result->success = false;
        }
ret:
        if(result->success)
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Successfully bag to %s", result->filename.c_str());
        }
        else
        {
            goal_handle->abort(result);
            RCLCPP_INFO(get_logger(), "Failed to bag: %s", result->status.c_str());
        }
        
        streaming = true;
    }

    void subscribe()
    {   
        auto topic_names_and_types = get_topic_names_and_types();
        for (auto& [topic, info] : subscriber_list) 
        {
            auto& [time, subscribed] = info;
          
            if(!subscribed)
            {
                auto it = topic_names_and_types.find(topic);
                if(it != topic_names_and_types.end())
                {
                    successful_subscription_count++;
                    subscribed = create_generic_subscription(
                        topic,
                        it->second[0],
                        rclcpp::QoS(10),
                        [this, topic](std::shared_ptr<rclcpp::SerializedMessage> msg){
                            bagger_callback(msg, topic);
                        }
                    );
                }
            }
        }
    }

    void add_env_var(const std::vector<std::string>& var)
    {
        for(auto& topic : var)
        {
            subscriber_list.emplace(std::move(topic), std::make_pair(stream_time, nullptr));
        }   
    }

    void get_params()
    {
        boost::process::environment env = boost::this_process::environment();
        auto it = env.find("BAG_DIR");
        if(it != env.end())
        {
            dir = it->to_string();
        }
        else
        {
            it = env.find("HOME");
            if(it != env.end())
            {
                dir = std::filesystem::path(it->to_string()) / "bag";
            }
            else
            {
                RCLCPP_FATAL(get_logger(), "Failed to obtain the bag package path, exiting...");
                rclcpp::shutdown();
            }
        }

        dir = std::filesystem::path(declare_parameter("bag_package_path", dir.string()));


        stream_time = declare_parameter("stream_time", 30);
        resubscribe_period = declare_parameter("resubscribe_period", 3.0);
        dated_folder = declare_parameter("dated_folder", true);

        std::vector<std::string> topics_param;
        for(const auto& [topic, types] : get_topic_names_and_types())
        {
            topics_param.push_back(topic);
        }
        topics_param = declare_parameter("topics", std::vector<std::string>(topics_param));

        for (const std::string& topic : topics_param) 
        {
            std::istringstream iss(topic);
            std::string token;
            std::vector<std::string> tokens;
            while (std::getline(iss, token, ',')) 
            {
                tokens.push_back(std::move(token));
            }

            size_t topic_stream_time =  stream_time;
            if(tokens.size() == 2)
            {
                try
                {
                    topic_stream_time = std::stoul(tokens[1]);
                }
                catch(const std::exception& e)
                {
                    RCLCPP_WARN(get_logger(), "Failed to set stream time %s for topic %s.", tokens[0].c_str(), tokens[1].c_str());
                }
            }

            subscriber_list.emplace(std::move(tokens[0]), std::make_pair(topic_stream_time, nullptr));
        }

        it = env.find("BAG_ALWAYS");
        if(it != env.end())
        {
            add_env_var(it->to_vector());
        }

        for(const auto& entry : env)
        {
            std::string key = entry.get_name();
            if (key.size() >= 4 && key.substr(0, 4) == "bag_") 
            {
                add_env_var(entry.to_vector());
            }
        }

        RCLCPP_INFO(get_logger(), "Default stream_time: %ld seconds", stream_time);
        RCLCPP_INFO(get_logger(), "Bag Directory: %s", dir.c_str());
    }
};
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto server = std::make_shared<online_bagger::Server>();
    rclcpp::spin(server);
    rclcpp::shutdown();

    return 0;
}



