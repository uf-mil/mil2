#include <boost/dll/alias.hpp>
#include <map>
#include <vector>

#include "mil_preflight/plugin.h"
#include "subjugator_msgs/msg/thruster_cmd.hpp"

namespace mil_preflight
{
class ActuatorPlugin : public PluginBase
{
  public:
    ActuatorPlugin()
    {
    }

    ~ActuatorPlugin()
    {
    }

    static std::shared_ptr<ActuatorPlugin> create()
    {
        return std::shared_ptr<ActuatorPlugin>(new ActuatorPlugin());
    }

  private:
    // std::vector<std::string> nodes_;
    std::string summery_;

    bool runAction(std::vector<std::string>&& parameters) final
    {
        std::string question =
            "Ensure that all fingers are clear of the area!\nIs it safe to operate the actuator: " + parameters[0] +
            " ?";
        if (askQuestion(question, { "Yes", "No" }) != 0)
        {
            summery_ = "User did not clear the area";
            return false;
        }

        std::vector<rclcpp::TopicEndpointInfo> infos = get_subscriptions_info_by_topic(parameters[1]);
        if (infos.size() == 0)
        {
            summery_ = "No subscriber subscribe to the topic: " + parameters[1];
            return false;
        }

        std::string& topicType = infos[0].topic_type();
        if (topicType != "subjugator_msgs/msg/Thruster Command")
        {
            summery_ = "Mismatched message type : " + infos[0].topic_type();
            return false;
        }

        subjugator_msgs::msg::ThrusterCmd cmd;
        cmd.name = parameters[2];

        try
        {
            cmd.thrust = std::stof(parameters[3]);
        }
        catch (std::exception const& e)
        {
            summery_ = "Invalid thrust parameter: " + parameters[3];
            return false;
        }

        rclcpp::Publisher<subjugator_msgs::msg::ThrusterCmd>::SharedPtr pub =
            create_publisher<subjugator_msgs::msg::ThrusterCmd>(parameters[1], 10);

        pub->publish(cmd);

        if (askQuestion("Had the " + parameters[2] + "start spinning?", { "Yes", "No" }) != 0)
        {
            summery_ = "User said the thruster didn't spin";
            return false;
        }

        cmd.thrust = 0;

        pub->publish(cmd);

        summery_ = "success";
        return true;
    }

    std::string const& getSummery() final
    {
        return summery_;
    }
};
}  // namespace mil_preflight

BOOST_DLL_ALIAS(mil_preflight::ActuatorPlugin::create, actuator_plugin);
