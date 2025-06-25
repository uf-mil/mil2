#include "Torpedoes.hh"

#include "gz/plugin/Register.hh"  // For GZ_ADD_PLUGIN

#include <gz/common/Console.hh>

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
    // Gather necessary information for future GatherWorldInfo() call
    this->sub9_Entity = entity;

    // Create copy of the Sub SDF element to perform modifications due to const
    this->sub9_SDF = sdf->Clone();

    // Subscribe to Keyboard events if needed

    // Initialize the ROS node and publisher
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
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

    // Prepare the factory message
    gz::msgs::EntityFactory factoryMsg;
    factoryMsg.set_sdf(sdfString);

    // Set the pose to X, Y, Z, and roll, pitch, yaw
    gz::msgs::Set(factoryMsg.mutable_pose(), gz::math::Pose3d(1.0 + torpedoCount, 1.0, 1.0, 0, 0, 0));

    // Send the request using the four-argument version for feedback
    std::string service = "/world/" + worldName + "/create";
    bool executed = node.Request(service, factoryMsg, timeout, reply, result);
    std::cout << "[Torpedoes] Request sent to service: " << service << std::endl;
    // std::cout << "[Torpedoes] Executed State: " << executed << std::endl;
    // std::cout << "[Torpedoes] Result: " << result << std::endl;
    // std::cout << "[Torpedoes] Reply: " << reply.DebugString() << std::endl;
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
                std::cout << "[Torpedoes] World Name Found: " << worldName << std::endl;
                worldNameFound = true;
                return false;  // Stop after finding the first world
            });
    }

    // Spawn two torpedoes maximum
    if (torpedoCount < 2)
    {
        std::cout << "[Torpedoes] Torpedo Count: " << torpedoCount + 1 << std::endl;
        this->SpawnTorpedo(this->worldName, this->Torpedo_sdfPath);
        torpedoCount++;
    }
}

}  // namespace torpedoes

// Register plugin for Gazebo
GZ_ADD_PLUGIN(torpedoes::Torpedoes, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)

// TODO:
// - Adjust torpedo.sdf so they don't fall through the world (look into gz::sim::systems)
// - Add keyboard event handling to spawn torpedoes on key press
// - Spawn torpedoes in relative to Sub9 position and orientation
// - Send out second torpedo slightly offset from the first one
// - Spawn torpedoes with initial velocity
// - Add logic to handle torpedo lifecycle (e.g., timeout, removal)
// - Move as much logic as possible to Configure()
