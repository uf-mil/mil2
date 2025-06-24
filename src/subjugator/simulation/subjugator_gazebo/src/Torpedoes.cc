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

    // Get the Torpedo SDF & Entity from ECM

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

    // Prepare the factory message
    gz::msgs::EntityFactory factoryMsg;
    factoryMsg.set_sdf(sdfString);

    // Optionally, set the pose if you want to spawn at a specific location
    // gz::msgs::Set(factoryMsg.mutable_pose(), gz::math::Pose3d(x, y, z, roll, pitch, yaw));

    // Send the request
    gz::transport::Node node;
    std::string service = "/world/" + worldName + "/create";
    bool result = node.Request(service, factoryMsg, 1000);
    if (!result)
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
        std::cout << "[Torpedoes] Torpedo Count: " << torpedoCount << std::endl;
        // this->SpawnTorpedo(this->worldName, sdfPath);
        torpedoCount++;
    }
}

}  // namespace torpedoes

// Register plugin for Gazebo
GZ_ADD_PLUGIN(torpedoes::Torpedoes, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
