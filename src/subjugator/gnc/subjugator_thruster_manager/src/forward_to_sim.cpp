#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "subjugator_msgs/msg/thruster_efforts.hpp"

class SimulatedThrusterCmds : public rclcpp::Node
{
  public:
    SimulatedThrusterCmds() : Node("simulate_thruster_cmds")
    {
        std::vector<std::string> thruster_topics = { "thruster/FLH", "thruster/FRH", "thruster/BLH", "thruster/BRH",
                                                     "thruster/FLV", "thruster/FRV", "thruster/BLV", "thruster/BRV" };

        for (auto const &topic : thruster_topics)
        {
            publishers_.push_back(this->create_publisher<std_msgs::msg::Float64>(topic, 10));
        }

        auto topic_callback = [this](subjugator_msgs::msg::ThrusterEfforts::UniquePtr msg) -> void
        {
            std::vector<double> thrust_values = { msg->thrust_flh, msg->thrust_frh, msg->thrust_blh, msg->thrust_brh,
                                                  msg->thrust_flv, msg->thrust_frv, msg->thrust_blv, msg->thrust_brv };

            for (size_t i = 0; i < publishers_.size(); ++i)
            {
                std_msgs::msg::Float64 msg_thruster;
                msg_thruster.data = thrust_values[i];
                publishers_[i]->publish(msg_thruster);
            }
        };

        subscription_ =
            this->create_subscription<subjugator_msgs::msg::ThrusterEfforts>("thruster_efforts", 10, topic_callback);
    }

  private:
    rclcpp::Subscription<subjugator_msgs::msg::ThrusterEfforts>::SharedPtr subscription_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulatedThrusterCmds>());
    rclcpp::shutdown();
    return 0;
}
