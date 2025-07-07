#include "Torpedoes.hh"

#include "gz/plugin/Register.hh"  // For GZ_ADD_PLUGIN

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
    // std::cout << "[Torpedoes] Sub9 Entity Name: "
    //           << ecm.Component<gz::sim::components::Name>(entity)->Data() << std::endl;

    // Create copy of the Sub SDF element to perform modifications due to const
    this->sub9_SDF = sdf->Clone();
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

    // Prepare the factory message
    gz::msgs::EntityFactory factoryMsg;
    factoryMsg.set_sdf(sdfString);

    // Set the pose to X, Y, Z, and roll, pitch, yaw
    gz::msgs::Set(factoryMsg.mutable_pose(), gz::math::Pose3d(sub9_pose.X(), sub9_pose.Y(), sub9_pose.Z() - 0.5,
                                                              sub9_pose.Roll(), sub9_pose.Pitch(), sub9_pose.Yaw()));

    // Send the request using the four-argument version for feedback
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

    if (!worldNameFound)
    {
        ecm.Each<gz::sim::components::World, gz::sim::components::Name>(
            [&](gz::sim::Entity const &entity, gz::sim::components::World const *,
                gz::sim::components::Name const *name) -> bool
            {
                this->worldName = name->Data();
                worldNameFound = true;
                return false;  // Stop after finding the first world
            });
    }

    // Spin the ROS2 node
    rclcpp::spin_some(torpedo_node_);

    // Get the pose of the sub9 entity
    auto sub9_component = ecm.Component<gz::sim::components::Pose>(this->sub9_Entity);
    if (sub9_component)
    {
        this->sub9_pose = sub9_component->Data();
        // std::cout << "[Torpedoes] Sub9 Pose: " << this->sub9_pose << std::endl;
    }

    // Spawn two torpedoes maximum, only once per 't' keypress
    if (torpedoCount < 2 && t_pressed)
    {
        this->SpawnTorpedo(this->worldName, this->Torpedo_sdfPath);
        torpedoCount++;
        t_pressed = false;  // Reset the flag after spawning
        std::cout << "[Torpedoes] Torpedo Count: " << torpedoCount << std::endl;
    }
}

}  // namespace torpedoes

// Register plugin for Gazebo
GZ_ADD_PLUGIN(torpedoes::Torpedoes, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate,
              gz::sim::ISystemPreUpdate)

// TODO:
// - Send out second torpedo slightly offset from the first one
// - Add logic to handle torpedo lifecycle (e.g., timeout, removal)
// - Move as much logic as possible to Configure()
