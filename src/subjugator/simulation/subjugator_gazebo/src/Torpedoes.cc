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
    this->modelEntity_ = entity;

    // Create copy of the Sub SDF element to perform modifications due to const
    this->sub9_SDF = sdf->Clone();

    // Initialize the ROS node and publisher
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
}

void Torpedoes::PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm)
{
    //  Check if the simulation is paused or running
    if (info.paused)
    {
        return;
    }
}

}  // namespace torpedoes

// Register plugin for Gazebo
GZ_ADD_PLUGIN(torpedoes::Torpedoes, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
