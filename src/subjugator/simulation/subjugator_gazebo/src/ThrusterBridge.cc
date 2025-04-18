#include "ThrusterBridge.hh"

#include "gz/plugin/Register.hh"  // For GZ_ADD_PLUGIN

#include <gz/common/Console.hh>

namespace thrusterBridge
{

ThrusterBridge::ThrusterBridge()
{
}

ThrusterBridge::~ThrusterBridge()
{
}

// Configure() -  Create ROS node that subscribes to /thruster_efforts & setup Gazebo Publishers //
void ThrusterBridge::Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                               gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr)
{
    // Initialize the ROS node and publisher
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    // Create a ROS2 node for publishing
    this->thrustNode = std::make_shared<rclcpp::Node>("thruster_bridge_node");
    this->thrustSubscription = this->thrustNode->create_subscription<subjugator_msgs::msg::ThrusterEfforts>(
        "/thruster_efforts", 1, std::bind(&thrusterBridge::ThrusterBridge::receiveEffortCallback, this, _1));

    // Gazebo Publishers
    std::vector<std::string> thrusterNames = { "FLH", "FRH", "BLH", "BRH", "FLV", "FRV", "BLV", "BRV" };
    for (auto const &thruster : thrusterNames)
    {
        // Full topic name for Gazebo with msg type gz::msgs::Double
        std::string topicName = "/model/" + thruster + "/joint/" + thruster + "_dir_joint/cmd_thrust";
        auto pub = gz_node.Advertise<gz::msgs::Double>(topicName);
        publishers[thruster] = pub;
    }

    // Get Thruster Force Parameters
    this->max_force_pos = this->thrustNode->get_parameter("max_force_pos").as_double();
    this->max_force_neg = this->thrustNode->get_parameter("max_force_neg").as_double();
}

// receiveEffortCallback() -  Whenever ROS2 node receives message call this //
// Update thrusterEfforts Map and publish to Gazebo //
void ThrusterBridge::receiveEffortCallback(subjugator_msgs::msg::ThrusterEfforts const &msg)
{
    // Process the received thruster efforts message
    // std::cout << "[ThrusterBridge] Received Thruster Efforts: " << std::endl;

    thrusterEfforts["FLH"] = msg.thrust_flh;
    thrusterEfforts["FRH"] = msg.thrust_frh;
    thrusterEfforts["BLH"] = msg.thrust_blh;
    thrusterEfforts["BRH"] = msg.thrust_brh;
    thrusterEfforts["FLV"] = msg.thrust_flv;
    thrusterEfforts["FRV"] = msg.thrust_frv;
    thrusterEfforts["BLV"] = msg.thrust_blv;
    thrusterEfforts["BRV"] = msg.thrust_brv;

    std::cout << "Thruster Values into Gazebo: " << std::endl;
    // Print the thrusterEfforts Map
    // for (const auto &[thrusterName, thrustValue] : thrusterEfforts)
    // {
    //     std::cout << "Thruster: " << thrusterName << ", Thrust Value: " << thrustValue << std::endl;
    // }
    // std::cout << std::endl;

    // Iterate through thrusterEfforts Map
    for (auto const &[thrusterName, thrustValue] : thrusterEfforts)
    {
        gz::msgs::Double thrustMsg;
        double scaledThrust = (thrustValue > 0) ? thrustValue / this->max_force_pos : thrustValue / this->max_force_neg;
        thrustMsg.set_data(scaledThrust);
        publishers[thrusterName].Publish(thrustMsg);

        // std::cout << "Publishing to: /model/" << thrusterName << "/joint/" << thrusterName << "_dir_joint/cmd_thrust"
        // << std::endl; std::cout << "Thrust Value: " << thrustValue << std::endl;
    }
}

// PostUpdate() -  Run the ROS node //
void ThrusterBridge::PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm)
{
    //  Check if the simulation is paused or running
    if (info.paused)
    {
        return;
    }

    // Spin the ROS node
    rclcpp::spin_some(this->thrustNode);
}

}  // namespace thrusterBridge

// Register plugin for Gazebo
GZ_ADD_PLUGIN(thrusterBridge::ThrusterBridge, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
