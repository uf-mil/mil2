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

// Configure() -  //
void ThrusterBridge::Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                               gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr)
{
    std::cout << std::endl;
    std::cout << "KEITHTHHHHHH" << std::endl;
    std::cout << std::endl;

    // Initialize the ROS node and publisher
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    // Create a ROS2 node for publishing
    this->thrustNode = std::make_shared<rclcpp::Node>("thruster_bridge_node");
    this->thrustSubscription = this->thrustNode->create_subscription<subjugator_msgs::msg::ThrusterEfforts>(
        "/thruster_efforts", 1, std::bind(&thrusterBridge::ThrusterBridge::receiveEffortCallback, this, _1));

    // Gazebo transport node
    std::function<void(gz::msgs::Double const &)> callback =
        std::bind(&ThrusterBridge::receiveGazeboCallback, this, _1);

    gz_node.Advertise("/model/FLH/joint/FLH_dir_joint/cmd_thrust", callback);
}

void ThrusterBridge::receiveEffortCallback(subjugator_msgs::msg::ThrusterEfforts const &msg)
{
    // Process the received thruster efforts message
    std::cout << "[ThrusterBridge] Received Thruster Efforts: " << std::endl;

    flh = msg.thrust_flh;
    frh = msg.thrust_frh;
    blh = msg.thrust_blh;
    brh = msg.thrust_brh;
    flv = msg.thrust_flv;
    frv = msg.thrust_frv;
    blv = msg.thrust_blv;
    brv = msg.thrust_brv;

    for (int i = 0; i < 8; i++)
    {
        std::string topicName = "/model/ " + thrusterNames[i] + "/joint/" + thrusterNames[i] + "_dir_joint/cmd_thrust";
        gz_node.Advertise(topicName, )
    }

    std::cout << "KEITHING IT" << std::endl;
}

void ThrusterBridge::receiveGazeboCallback(gz::msgs::Double const &msg)
{
    // Process the received thruster efforts message
    std::cout << "[Gazebo Node] Received Thruster Efforts: " << std::endl;

    msg = flh

    // flh = msg.thrust_flh;
    // frh = msg.thrust_frh;
    // blh = msg.thrust_blh;
    // brh = msg.thrust_brh;
    // flv = msg.thrust_flv;
    // frv = msg.thrust_frv;
    // blv = msg.thrust_blv;
    // brv = msg.thrust_brv;
}

// PostUpdate() -  //
void ThrusterBridge::PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm)
{
    //  Check if the simulation is paused or running
    if (info.paused)
    {
        return;
    }

    // Spin the ROS node to process incoming messages
    rclcpp::spin_some(this->thrustNode);
}

}  // namespace thrusterBridge

// Register plugin for Gazebo
GZ_ADD_PLUGIN(thrusterBridge::ThrusterBridge, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
