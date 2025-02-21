#include "Hydrophone.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>  // For GZ_ADD_PLUGIN

Hydrophone::Hydrophone()
{
}

Hydrophone::~Hydrophone()
{
}

void Hydrophone::Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                           gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr)
{
  this->modelEntity_ = entity;

  std::string pingerPrefix = "Pinger_";

  // Iterate through all entities that have a Pose component
  ecm.Each<gz::sim::components::Pose, gz::sim::components::Name>(
      [&](gz::sim::Entity const &ent, gz::sim::components::Pose const *pose,
          gz::sim::components::Name const *name) -> bool
      {
        if (!pose || !name)
        {
          return true;  // Skip if either component is missing
        }
        std::string entityName = name->Data();  // Get entity name

        // Check if name starts with "Pinger_"
        if (entityName.find(pingerPrefix) == 0)
        {
          this->pingerLocations_.push_back(pose->Data());  // Store Pose
          this->pingerCount_++;                            // Increment counter
          gzdbg << "[Hydrophone] Found pinger [" << entityName << "] at pose [" << pose->Data() << "]\n";
        }

        return true;  // Continue iteration
      }

  );

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  this->rosNode_ = rclcpp::Node::make_shared("hydrophone_node");
  this->depthPub_ = this->rosNode_->create_publisher<mil_msgs::msg::DepthStamped>("/depth", 10);

  gzdbg << "[Hydrophone] Configure() done." << "\n"
        << "Entity = " << entity << "\n"
        << "Pose = " << this->pose_ << "\n";
}

void Hydrophone::PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm)
{
  // Check if the simulation is paused
  if (info.paused)
  {
    return;
  }

  // Spin the ROS node to process incoming messages
  rclcpp::spin_some(this->rosNode_);

  // Update values of hydrophones on update
  auto poseComp = ecm.Component<gz::sim::components::Pose>(this->modelEntity_);
  if (!poseComp)
  {
    gzerr << "[Hydrophone] Pose component not found for entity [" << this->modelEntity_ << "]\n";
    return;
  }

  // Get the current pose of the hydrophone
  gz::math::Pose3d hydroPose = poseComp->Data();

  // Clear previous difference vectors
  this->pingerDiffs_.clear();

  // Iterate over all pinger locations and compute difference vectors
  for (auto const &pingerPose : this->pingerLocations_)
  {
    // Compute difference vector (pinger - hydrophone)
    gz::math::Vector3d diffVector = pingerPose.Pos() - hydroPose.Pos();
    this->pingerDiffs_.push_back(diffVector);

    // Create ROS2 message
    // Msg for Origin Direction Body
    mil_msgs::msg::ProcessedPing msg;
    msg.origin_direction_body.x = diffVector.X();
    msg.origin_direction_body.y = diffVector.Y();
    msg.origin_direction_body.z = diffVector.Z();

    // Msg for Pinger Frequency
    msg.frequency = 1000;  // Example frequency, replace with actual value

    // Msg for Origin Distance
    msg.origin_distance_m = diffVector.Length();

    // Publish message
    this->pingPub_->publish(msg);

    gzdbg << "[Hydrophone] Pinger at [" << pingerPose.Pos() << "] -> Difference vector: [" << diffVector << "]\n";
  }
}

// Register plugin so Gazebo can see it
GZ_ADD_PLUGIN(hydrophone::Hydrophone, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
