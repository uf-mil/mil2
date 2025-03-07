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

void Hydrophone::Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                           gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr)
{
  std::cout << std::endl;
  std::cout << "Entered Configure" << std::endl;

  // Gather necessary information for future GatherWorldInfo() call
  this->modelEntity_ = entity;
  std::string pingerPrefix = "Pinger_";

  // Create copy of the SDF element to perform modifications due to const
  this->sdf_ = sdf->Clone();
  // std::string testing = sdfCopy->Get<std::string>("model");
  // std::cout << "Model Name: " << testing << std::endl;

  // Iterate through sdf file, getting all <frequency> elements
  // !! Does not enter Loop - Because 'frequency' doesn't exist in sub.sdf!!
  // auto frequencyElement = sdfCopy->GetElement("frequency");
  // std::cout << "Frequency Element: " << frequencyElement << std::endl;
  // while (frequencyElement)
  // {
  //   // Get and store frequency
  //   double frequency = frequencyElement->Get<double>();
  //   std::cout << "Frequency: " << frequency << std::endl;
  //   pingerFrequencies_.push_back(frequency);

  //   // Move to the next <frequency> element
  //   frequencyElement = frequencyElement->GetNextElement("frequency");
  // }

  size_t entityCount = ecm.EntityCount();
  std::cout << "Entity Count: " << entityCount << std::endl;

  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  this->rosNode_ = std::make_shared<rclcpp::Node>("hydrophone_node");
  this->pingPub_ = this->rosNode_->create_publisher<mil_msgs::msg::ProcessedPing>("/ping", 1);

  std::cout << "[Hydrophone] Configure() done." << "\n"
            << "Entity = " << entity << "\n"
            << "Pose = " << this->pose_ << std::endl;
  std::cout << std::endl;
}

void Hydrophone::GatherWorldInfo(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                                 gz::sim::EntityComponentManager const &_ecm)
{
  std::string pingerPrefix = "Pinger_";

  auto frequencyElement = this->sdf_->FindElement("frequency");
  while (frequencyElement)
  {
    // Get and store frequency
    double frequency = frequencyElement->Get<double>();
    std::cout << "Frequency: " << frequency << std::endl;
    pingerFrequencies_.push_back(frequency);

    // Move to the next <frequency> element
    frequencyElement = frequencyElement->GetNextElement("frequency");
  }

  size_t entityCount = _ecm.EntityCount();
  std::cout << "Entity Count: " << entityCount << std::endl;
  // Iterate through all entities that have a Pose component
  _ecm.Each<gz::sim::components::Pose, gz::sim::components::Name>(
      [&](gz::sim::Entity const &ent, gz::sim::components::Pose const *pose,
          gz::sim::components::Name const *name) -> bool
      {
        // std::cout << "Inside Each" << std::endl;
        if (!pose || !name)
        {
          std::cout << "No pose or name" << std::endl;
          return true;  // Skip if either component is missing
        }
        std::string entityName = name->Data();  // Get entity name
        std::cout << "Entity Name: " << entityName << std::endl;
        // Check if name starts with "Pinger_"
        if (entityName.find(pingerPrefix) == 0)
        {
          // Store the pose of the pinger
          this->pingerLocations_.push_back(pose->Data());  // Store Pose
          std::cout << "Pinger Pose X: " << pose->Data().Pos().X() << std::endl;
          std::cout << "Pinger Pose Y: " << pose->Data().Pos().Y() << std::endl;
          std::cout << "Pinger Pose Z: " << pose->Data().Pos().Z() << std::endl;
          this->pingerCount_++;  // Increment counter

          // Store the name of the pinger
          this->pingerNames_.push_back(entityName);  // Store name

          // Assign the extracted frequencies
          // std::cout << "Pinger Frequencies Count: " << pingerFrequencies_.size() << std::endl;
          for (double freq : pingerFrequencies_)
          {
            this->pingerFrequencies_.push_back(freq);
            std::cout << "[Hydrophone] Found pinger [" << entityName << "] with frequency [" << freq << "] at pose ["
                      << pose->Data() << "]" << std::endl;
          }
        }

        return true;  // Continue iteration
      }

  );
  std::cout << "Gathered World Info" << std::endl;
}

void Hydrophone::PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm)
{
  // std::cout << "Post Update Entered" << std::endl;
  //  Check if the simulation is paused
  size_t entityCount = ecm.EntityCount();
  if (!worldGathered)
  {
    GatherWorldInfo(this->modelEntity_, this->sdf_, ecm);
    std::cout << "World Gathered" << std::endl;
    worldGathered = true;
  }

  if (info.paused)
  {
    // std::cout << "Simulation is paused" << std::endl;
    return;
  }

  // Spin the ROS node to process incoming messages
  rclcpp::spin_some(this->rosNode_);

  // Update values of hydrophones on update
  // auto poseComp = ecm.Component<gz::sim::components::Pose>(this->modelEntity_);
  // if (!poseComp)
  // {
  //   gzerr << "[Hydrophone] Pose component not found for entity [" << this->modelEntity_ << "]\n";
  //   return;
  // }

  // // Get the current pose of the hydrophone
  // gz::math::Pose3d hydroPose = poseComp->Data();

  // // Clear previous difference vectors
  // this->pingerDiffs_.clear();

  // // Iterate over all pinger locations and compute difference vectors
  // for (auto const &pingerPose : this->pingerLocations_)
  // {
  //   int i = 0;
  //   // Compute difference vector (pinger - hydrophone)
  //   gz::math::Vector3d diffVector = pingerPose.Pos() - hydroPose.Pos();
  //   this->pingerDiffs_.push_back(diffVector);

  //   // Create ROS2 message
  //   // Msg for Origin Direction Body
  //   mil_msgs::msg::ProcessedPing msg;
  //   msg.origin_direction_body.x = diffVector.X();
  //   msg.origin_direction_body.y = diffVector.Y();
  //   msg.origin_direction_body.z = diffVector.Z();

  //   // Msg for Pinger Frequency
  //   msg.frequency = this->pingerFrequencies_.at(i);

  //   // Msg for Origin Distance
  //   msg.origin_distance_m = diffVector.Length();

  //   // Publish message
  //   this->pingPub_->publish(msg);

  //   std::cout << "[Hydrophone] Pinger at [" << pingerPose.Pos() << "] -> Difference vector: [" << diffVector << "]"
  //             << "with frequency [" << this->pingerFrequencies_.at(i) << "]" << std::endl;

  //   i++;
  // }
}

}  // namespace hydrophone

// Register plugin so Gazebo can see it
GZ_ADD_PLUGIN(hydrophone::Hydrophone, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
