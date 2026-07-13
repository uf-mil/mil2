#include "context.hpp"

#include <subjugator_msgs/srv/admission.hpp>

class Admission : public BT::StatefulActionNode
{
    rclcpp::Client<subjugator_msgs::srv::Admission>::SharedPtr client;
    rclcpp::Client<subjugator_msgs::srv::Admission>::SharedFuture future;

public:
    Admission(const std::string &name, const BT::NodeConfig &config) :
        BT::StatefulActionNode(name, config) {
        static auto client = node->create_client<subjugator_msgs::srv::Admission>("admission");
        this->client = client;
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("mission", "")
        };
    }

    BT::NodeStatus onStart() override {
        auto req =
            std::make_shared<subjugator_msgs::srv::Admission::Request>();
        getInput("mission", req->name);
        future = client->async_send_request(req).share();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            if (future.get()->success) {
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
    }
};

REGISTER(Admission)
