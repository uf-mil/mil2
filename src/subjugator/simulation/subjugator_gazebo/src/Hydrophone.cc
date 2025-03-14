#include "Hydrophone.hh"

#include <gz/common/Console.hh>

#include "gz/plugin/Register.hh"  // For GZ_ADD_PLUGIN

namespace hydrophone
{

Hydrophone::Hydrophone()
{
}

Hydrophone::~Hydrophone()
{
}

void Hydrophone::FormatPinger(std::string &entityName)
{
  // Input string is of format: Pinger_1X_00000/Pinger_2X_00000
  // Where X can be a value from A-Z and 00000 is a 5 digit frequency

  // Split input string by underscore
  size_t lastUnderscore = entityName.find_last_of('_');

  // Extract the frequency after the last underscore and convert to an integer
  std::string frequencyStr = entityName.substr(lastUnderscore + 1);
  int frequency = std::stoi(frequencyStr);

  // Extract the name of the Pinger
  std::string pingerName = entityName.substr(0, lastUnderscore);

  // Input Frequency and Name to the respective vectors
  this->pingerFrequencies_.push_back(frequency);
  this->pingerNames_.push_back(pingerName);

  // Print the extracted frequency and name
  // std::cout << "[Hydrophone] Extracted Frequency: " << frequency << std::endl;
  // std::cout << "[Hydrophone] Extracted Name: " << pingerName << std::endl;
}

void Hydrophone::Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                           gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr)
{
  // Gather necessary information for future GatherWorldInfo() call
  this->modelEntity_ = entity;

  // Create copy of the SDF element to perform modifications due to const
  this->sdf_ = sdf->Clone();

  size_t entityCount = ecm.EntityCount();

  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  this->rosNode_ = std::make_shared<rclcpp::Node>("hydrophone_node");
  this->pingPub_ = this->rosNode_->create_publisher<mil_msgs::msg::ProcessedPing>("/ping", 1);
}

void Hydrophone::GatherWorldInfo(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                                 gz::sim::EntityComponentManager const &_ecm)
{
  std::string pingerPrefix = "Pinger_";

  size_t entityCount = _ecm.EntityCount();
  // std::cout << "Entity Count: " << entityCount << std::endl;
  // Iterate through all entities that have a Pose component
  _ecm.Each<gz::sim::components::Pose, gz::sim::components::Name>(
      [&](gz::sim::Entity const &ent, gz::sim::components::Pose const *pose,
          gz::sim::components::Name const *name) -> bool
      {
        // std::cout << "Inside Each" << std::endl;
        if (!pose || !name)
        {
          // std::cout << "No pose or name" << std::endl;
          return true;  // Skip if either component is missing
        }
        if (name->Data() == "hydrophone_sensor_link")
        {
          this->hydrophoneEntity_ = ent;
        }
        std::string entityName = name->Data();  // Get entity name
        // std::cout << "Entity Name: " << entityName << std::endl;
        // Check if name starts with "Pinger_"
        if (entityName.find(pingerPrefix) == 0)
        {
          // Store the pose of the pinger
          this->pingerLocations_.push_back(pose->Data());  // Store Pose
          // std::cout << "Pinger Pose X: " << pose->Data().Pos().X() << std::endl;
          // std::cout << "Pinger Pose Y: " << pose->Data().Pos().Y() << std::endl;
          // std::cout << "Pinger Pose Z: " << pose->Data().Pos().Z() << std::endl;

          // Format the name to extract frequency and Name
          FormatPinger(entityName);
        }

        return true;  // Continue iteration
      }

  );
  // std::cout << "Gathered World Info" << std::endl;
}

void Hydrophone::PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm)
{
  //  Check if the simulation is paused
  if (info.paused)
  {
    return;
  }

  // Gather world info only once
  size_t entityCount = ecm.EntityCount();
  if (!worldGathered)
  {
    GatherWorldInfo(this->modelEntity_, this->sdf_, ecm);
    worldGathered = true;
    // std::cout << "World Gathered" << std::endl;
  }

  // Spin the ROS node to process incoming messages
  rclcpp::spin_some(this->rosNode_);

  // Get current pose of the Sub9 entity
  auto hydrophoneComponent = ecm.Component<gz::sim::components::Pose>(this->hydrophoneEntity_);
  if (hydrophoneComponent)
  {
    this->hydrophonePose_ = hydrophoneComponent->Data();
    // std::cout << "[Hydrophone] Hydrophone Pose Updated: " << this->hydrophonePose_ << std::endl;
  }

  // Iterate over all pinger locations and compute difference vectors
  int i = 0;
  for (auto const &pingerPose : this->pingerLocations_)
  {
    // Compute difference vector (pinger - hydrophone)
    gz::math::Vector3d diffVector = pingerPose.Pos() - hydrophonePose_.Pos();
    this->pingerDiffs_.push_back(diffVector);

    // Create ROS2 message
    // Msg for Origin Direction Body
    mil_msgs::msg::ProcessedPing msg;
    msg.origin_direction_body.x = diffVector.X();
    msg.origin_direction_body.y = diffVector.Y();
    msg.origin_direction_body.z = diffVector.Z();

    // Msg for Pinger Frequency
    msg.frequency = this->pingerFrequencies_.at(i);

    // Msg for Origin Distance
    msg.origin_distance_m = diffVector.Length();

    // Publish message
    this->pingPub_->publish(msg);

    // std::cout << "Pinger at [" << pingerPose.Pos() << "]" << std::endl
    //           << "Difference vector: [" << diffVector << "]" << std::endl
    //           << "Difference Length: [" << diffVector.Length() << "]" << std::endl
    //           << "Frequency: [" << this->pingerFrequencies_.at(i) << "]" << std::endl;

    i++;
  }
}

}  // namespace hydrophone

// Register plugin so Gazebo can see it
GZ_ADD_PLUGIN(hydrophone::Hydrophone, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
