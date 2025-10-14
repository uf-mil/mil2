#include "MarbleDropper.hh"

namespace marble_dropper
{

MarbleDropper::MarbleDropper()
{
}

MarbleDropper::~MarbleDropper()
{
}

void MarbleDropper::Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                              gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr)
{
    std::cout << "[MarbleDropper] Configuring MarbleDropper Plugin ..." << std::endl;

    // Initialize the ROS node and keypress subscriber
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
    marble_node_ = std::make_shared<rclcpp::Node>("marble_dropper_plugin_node");
    keypress_sub_ = marble_node_->create_subscription<std_msgs::msg::String>(
        "/keyboard/keypress", 10,
        std::bind(&marble_dropper::MarbleDropper::KeypressCallback, this, std::placeholders::_1));

    // Gather necessary information for future GatherWorldInfo() call
    this->sub9_Entity = entity;

    // Create copy of the Sub SDF element to perform modifications due to const
    this->sub9_SDF = sdf->Clone();

    if (!worldNameFound)
    {
        ecm.Each<gz::sim::components::World, gz::sim::components::Name>(
            [&](gz::sim::Entity const &entity, gz::sim::components::World const *,
                gz::sim::components::Name const *name) -> bool
            {
                this->worldName = name->Data();
                worldNameFound = true;
                return false;
            });
    }
    std::cout << "[MarbleDropper] World Name: " << this->worldName << std::endl;
}

void MarbleDropper::KeypressCallback(std_msgs::msg::String::SharedPtr const msg)
{
    std::cout << "[MarbleDropper] Keypress received: " << msg->data << std::endl;
    if (msg->data == "m" && !m_pressed)
    {
        m_pressed = true;
    }
}

void MarbleDropper::PreUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager &ecm)
{
    // Unnecessary if not adding velocity to marble(s)
}

void MarbleDropper::SpawnMarble(std::string const &worldName, std::string const &sdfPath)
{
    // Load SDF file as string
    std::ifstream sdfFile(sdfPath);
    std::stringstream buffer;
    buffer << sdfFile.rdbuf();
    std::string sdfString = buffer.str();

    // Remove XML declaration if present
    size_t xmlDeclPos = sdfString.find("<?xml");
    if (xmlDeclPos != std::string::npos)
    {
        size_t endDecl = sdfString.find("?>", xmlDeclPos);
        if (endDecl != std::string::npos)
        {
            sdfString.erase(xmlDeclPos, endDecl - xmlDeclPos + 2);
        }
    }

    // Make model name unique for each marble (assume double quotes in SDF)
    std::string uniqueName = "marble_" + std::to_string(marbleCount + 1);
    size_t namePos = sdfString.find("<model name=\"");
    if (namePos != std::string::npos)
    {
        size_t quotePos = sdfString.find("\"", namePos + 13);
        if (quotePos != std::string::npos)
        {
            sdfString.replace(namePos + 13, quotePos - (namePos + 13), uniqueName);
        }
    }
    // Track the marble model name for lookup in PreUpdate
    marbleModelNames.insert(uniqueName);

    // Record spawn time
    marbleSpawnTimes[uniqueName] = -1.0;  // Will be set to sim time in PostUpdate

    // Prepare the factory message
    gz::msgs::EntityFactory factoryMsg;
    factoryMsg.set_sdf(sdfString);

    // Set the pose to X, Y, Z, and roll, pitch, yaw
    // -0.5 offset in Z to not hit sub9
    gz::msgs::Set(factoryMsg.mutable_pose(), gz::math::Pose3d(sub9_pose.X(), sub9_pose.Y(), sub9_pose.Z() - 0.5,
                                                              sub9_pose.Roll(), sub9_pose.Pitch(), sub9_pose.Yaw()));

    // Send the request to create model in .world
    std::string service = "/world/" + worldName + "/create";
    bool executed = node.Request(service, factoryMsg, timeout, reply, result);
    if (!executed || !result)
    {
        gzerr << "Failed to spawn marble model." << std::endl;
    }
}

void MarbleDropper::PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm)
{
    //  Check if the simulation is paused or running
    if (info.paused)
    {
        return;
    }

    // Spin the ROS2 node
    rclcpp::spin_some(marble_node_);

    // Get the updated pose of the sub9 entity
    auto sub9_component = ecm.Component<gz::sim::components::Pose>(this->sub9_Entity);
    if (sub9_component)
    {
        this->sub9_pose = sub9_component->Data();
    }

    // Set spawn time for new marble_dropper (if not set)
    for (auto &pair : marbleSpawnTimes)
    {
        if (pair.second < 0.0)
        {
            pair.second = std::chrono::duration_cast<std::chrono::duration<double>>(info.simTime).count();
        }
    }

    // Remove marble_dropper after 8 seconds
    for (auto spawnedMarbIter = marbleSpawnTimes.begin(); spawnedMarbIter != marbleSpawnTimes.end();)
    {
        double timeElapsed =
            std::chrono::duration_cast<std::chrono::duration<double>>(info.simTime).count() - spawnedMarbIter->second;
        if (timeElapsed > 8.0)
        {
            // Remove entity by name
            gz::msgs::Entity removeMsg;
            removeMsg.set_name(spawnedMarbIter->first);
            removeMsg.set_type(gz::msgs::Entity::MODEL);
            std::string removeService = "/world/" + worldName + "/remove";
            node.Request(removeService, removeMsg, timeout, reply, result);

            // Clean up tracking
            marbleModelNames.erase(spawnedMarbIter->first);
            spawnedMarbIter = marbleSpawnTimes.erase(spawnedMarbIter);
        }
        else
        {
            ++spawnedMarbIter;
        }
    }

    // Spawn two marble_dropper maximum, only once per 'm' keypress
    if (marbleCount < 2 && m_pressed)
    {
        std::cout << "[MarbleDropper] Spawning marble #" << (marbleCount + 1) << "..." << std::endl;
        this->SpawnMarble(this->worldName, this->Marble_sdfPath);
        marbleCount++;
        m_pressed = false;  // Reset the flag after spawning
    }
}

}  // namespace marble_dropper

// Register plugin for Gazebo
GZ_ADD_PLUGIN(marble_dropper::MarbleDropper, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate,
              gz::sim::ISystemPreUpdate)
