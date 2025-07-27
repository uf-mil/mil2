#include "Torpedoes.hh"

namespace torpedoes
{

Torpedoes::Torpedoes()
{
}

Torpedoes::~Torpedoes()
{
}

void Torpedoes::Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                          gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr)
{
    std::cout << "[Torpedoes] Configuring Torpedoes Plugin ..." << std::endl;

    // Initialize the ROS node and keypress subscriber
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
    torpedo_node_ = std::make_shared<rclcpp::Node>("torpedoes_plugin_node");
    keypress_sub_ = torpedo_node_->create_subscription<std_msgs::msg::String>(
        "/keyboard/keypress", 10, std::bind(&torpedoes::Torpedoes::KeypressCallback, this, std::placeholders::_1));

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
}

void Torpedoes::KeypressCallback(std_msgs::msg::String::SharedPtr const msg)
{
    if (msg->data == "t" && !t_pressed)
    {
        t_pressed = true;
    }
}

void Torpedoes::PreUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager &ecm)
{
    // Only process torpedoes if they have been spawned
    if (torpedoCount == 0 || info.paused)
    {
        return;
    }

    // Only process torpedo models that have not had their velocity set
    for (auto const &modelName : torpedoModelNames)
    {
        if (torpedoesWithVelocitySet.count(modelName) > 0)
            continue;
        // Find the model entity by name
        auto modelEntity = ecm.EntityByComponents(gz::sim::components::Name(modelName));
        if (modelEntity == gz::sim::kNullEntity)
        {
            continue;
        }
        // Find the 'body' link child entity using ChildrenByComponents
        gz::sim::Entity bodyLink = gz::sim::kNullEntity;
        auto linkChildren = ecm.ChildrenByComponents(modelEntity, gz::sim::components::Link());
        for (auto linkEntity : linkChildren)
        {
            auto nameComp = ecm.Component<gz::sim::components::Name>(linkEntity);
            if (nameComp && nameComp->Data() == "body")
            {
                bodyLink = linkEntity;
                break;
            }
        }
        if (bodyLink != gz::sim::kNullEntity)
        {
            gz::sim::Link link(bodyLink);
            link.SetLinearVelocity(ecm, gz::math::Vector3d(10.0, 0.0, 0.0));
            // Check if velocity is nonzero, then mark as set
            auto velComp = ecm.Component<gz::sim::components::LinearVelocity>(bodyLink);
            if (velComp && velComp->Data().Length() > 0.1)
            {
                torpedoesWithVelocitySet.insert(modelName);
            }
        }
    }
}

void Torpedoes::SpawnTorpedo(std::string const &worldName, std::string const &sdfPath)
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

    // Make model name unique for each torpedo (assume double quotes in SDF)
    std::string uniqueName = "torpedo_" + std::to_string(torpedoCount + 1);
    size_t namePos = sdfString.find("<model name=\"");
    if (namePos != std::string::npos)
    {
        size_t quotePos = sdfString.find("\"", namePos + 13);
        if (quotePos != std::string::npos)
        {
            sdfString.replace(namePos + 13, quotePos - (namePos + 13), uniqueName);
        }
    }
    // Track the torpedo model name for lookup in PreUpdate
    torpedoModelNames.insert(uniqueName);

    // Record spawn time
    torpedoSpawnTimes[uniqueName] = -1.0;  // Will be set to sim time in PostUpdate

    // Prepare the factory message
    gz::msgs::EntityFactory factoryMsg;
    factoryMsg.set_sdf(sdfString);

    // Set the pose to X, Y, Z, and roll, pitch, yaw
    // -0.5 offset in Z to not hit sub9 but will replace once torp launcher model is added
    gz::msgs::Set(factoryMsg.mutable_pose(), gz::math::Pose3d(sub9_pose.X(), sub9_pose.Y(), sub9_pose.Z() - 0.5,
                                                              sub9_pose.Roll(), sub9_pose.Pitch(), sub9_pose.Yaw()));

    // Send the request to create model in .world
    std::string service = "/world/" + worldName + "/create";
    bool executed = node.Request(service, factoryMsg, timeout, reply, result);
    if (!executed || !result)
    {
        gzerr << "Failed to spawn torpedo model." << std::endl;
    }
}

void Torpedoes::PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm)
{
    //  Check if the simulation is paused or running
    if (info.paused)
    {
        return;
    }

    // Spin the ROS2 node
    rclcpp::spin_some(torpedo_node_);

    // Get the updated pose of the sub9 entity
    auto sub9_component = ecm.Component<gz::sim::components::Pose>(this->sub9_Entity);
    if (sub9_component)
    {
        this->sub9_pose = sub9_component->Data();
    }

    // Set spawn time for new torpedoes (if not set)
    for (auto &pair : torpedoSpawnTimes)
    {
        if (pair.second < 0.0)
        {
            pair.second = std::chrono::duration_cast<std::chrono::duration<double>>(info.simTime).count();
        }
    }

    // Remove torpedoes after 8 seconds
    for (auto spawnedTorpsIter = torpedoSpawnTimes.begin(); spawnedTorpsIter != torpedoSpawnTimes.end();)
    {
        double timeElapsed =
            std::chrono::duration_cast<std::chrono::duration<double>>(info.simTime).count() - spawnedTorpsIter->second;
        if (timeElapsed > 8.0)
        {
            // Remove entity by name
            gz::msgs::Entity removeMsg;
            removeMsg.set_name(spawnedTorpsIter->first);
            removeMsg.set_type(gz::msgs::Entity::MODEL);
            std::string removeService = "/world/" + worldName + "/remove";
            node.Request(removeService, removeMsg, timeout, reply, result);

            // Clean up tracking
            torpedoModelNames.erase(spawnedTorpsIter->first);
            torpedoesWithVelocitySet.erase(spawnedTorpsIter->first);
            spawnedTorpsIter = torpedoSpawnTimes.erase(spawnedTorpsIter);
        }
        else
        {
            ++spawnedTorpsIter;
        }
    }

    // Spawn two torpedoes maximum, only once per 't' keypress
    if (torpedoCount < 2 && t_pressed)
    {
        this->SpawnTorpedo(this->worldName, this->Torpedo_sdfPath);
        torpedoCount++;
        t_pressed = false;  // Reset the flag after spawning
    }
}

}  // namespace torpedoes

// Register plugin for Gazebo
GZ_ADD_PLUGIN(torpedoes::Torpedoes, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate,
              gz::sim::ISystemPreUpdate)
