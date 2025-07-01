#include <map>
#include <stdexcept>
#include <vector>

#include <boost/dll/alias.hpp>

#include "mil_preflight/plugin.h"
#include "subjugator_msgs/msg/thruster_efforts.hpp"

namespace mil_preflight
{
class ActuatorPlugin : public SimplePlugin
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
    subjugator_msgs::msg::ThrusterEfforts assignThrust(subjugator_msgs::msg::ThrusterEfforts cmd, std::string thruster,
                                                       float thrust)
    {
        if (thruster == "FLH")
            cmd.thrust_flh = thrust;

        else if (thruster == "FRH")
            cmd.thrust_frh = thrust;

        else if (thruster == "BLH")
            cmd.thrust_blh = thrust;

        else if (thruster == "BRH")
            cmd.thrust_brh = thrust;

        else if (thruster == "FLV")
            cmd.thrust_flv = thrust;

        else if (thruster == "FRV")
            cmd.thrust_frv = thrust;

        else if (thruster == "BLV")
            cmd.thrust_blv = thrust;

        else if (thruster == "BRV")
            cmd.thrust_brv = thrust;

        return cmd;
    }

    std::pair<bool, std::string> runAction(std::shared_ptr<Action> action) final
    {
        std::string question =
            "Ensure that all fingers are clear of the area!\nIs it safe to operate the actuator: " + action->getName() +
            " ?";

        if (action->onQuestion(std::move(question), { "Yes", "No" }).get() != 0)
        {
            return { false, "User did not clear the area" };
        }

        std::string const topic_name = action->getParameters()[0];
        std::vector<rclcpp::TopicEndpointInfo> infos = get_subscriptions_info_by_topic(topic_name);
        if (infos.size() == 0)
        {
            return { false, "No subscriber subscribing to the topic: " + topic_name };
        }

        std::string& topicType = infos[0].topic_type();
        if (topicType != "subjugator_msgs/msg/ThrusterEfforts")
        {
            return { false, "Mismatched message type : " + infos[0].topic_type() };
        }

        subjugator_msgs::msg::ThrusterEfforts cmd;
        std::string thruster = action->getParameters()[1];
        std::string thrust = action->getParameters()[2];

        try
        {
            cmd = this->assignThrust(cmd, thruster, std::stof(thrust));
        }
        catch (std::exception const& e)
        {
            return { false, "Invalid thrust parameter: " + thrust };
        }

        rclcpp::Publisher<subjugator_msgs::msg::ThrusterEfforts>::SharedPtr pub =
            create_publisher<subjugator_msgs::msg::ThrusterEfforts>(topic_name, 10);

        auto start_time = std::chrono::steady_clock::now();
        std::chrono::seconds timeout(2);  // TODO: Make into a variable
        int thruster_did_spin = 0;

        while (std::chrono::steady_clock::now() - start_time < timeout)
        {
            pub->publish(cmd);
        }

        cmd = this->assignThrust(cmd, thruster, 0);

        pub->publish(cmd);

        thruster_did_spin = action->onQuestion("Did the " + thruster + " thruster spin?", { "Yes", "No" }).get() == 0;

        if (!thruster_did_spin)
        {
            return { false, "User said the thruster didn't spin" };
        }

        return { true, "Success" };
    }
};
}  // namespace mil_preflight

BOOST_DLL_ALIAS(mil_preflight::ActuatorPlugin::create, actuator_plugin);
