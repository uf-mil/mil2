#include "RealPosition.hh"

#include <gz/msgs/pose.pb.h>

#include <iostream>

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

// Add plugins to gazebo simulator alongside its dependencies
GZ_ADD_PLUGIN(real_position::RealPosition, gz::sim::System, real_position::RealPosition::ISystemPostUpdate)

using namespace real_position;

RealPosition::RealPosition()
{
}

RealPosition::~RealPosition()
{
}

void RealPosition::Configure(gz::sim::Entity const &_entity, std::shared_ptr<sdf::Element const> const &_sdf,
                             gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventManager)
{
    // Store the entity ID
    this->entityId = _entity;

    // Get the entity name for logging and topic naming
    auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
    if (nameComp)
    {
        this->entityName = nameComp->Data();
    }
    else
    {
        this->entityName = "unknown_entity_" + std::to_string(_entity);
    }

    // Configure topic name - can be customized via SDF parameters
    if (_sdf->HasElement("topic"))
    {
        this->topicName = _sdf->Get<std::string>("topic");
    }
    else
    {
        this->topicName = "/gazebo/pose/" + this->entityName;
    }

    // Configure publish frequency if specified in SDF
    if (_sdf->HasElement("publish_frequency"))
    {
        this->publishFrequency = _sdf->Get<int>("publish_frequency");
    }

    // Create publisher
    this->posePub = this->node.Advertise<gz::msgs::Pose>(this->topicName);

    std::cout << "RealPosition initialized for entity: " << this->entityName << " (ID: " << _entity << ")" << std::endl;
    std::cout << "Publishing pose on topic: " << this->topicName << std::endl;
    std::cout << "Publish frequency: every " << this->publishFrequency << " updates" << std::endl;
}

void RealPosition::PostUpdate(gz::sim::UpdateInfo const &_info, gz::sim::EntityComponentManager const &_ecm)
{
    // Only runs code if the simulation is active
    if (!_info.paused)
    {
        // Increment update counter
        this->updateCounter++;

        // Only publish at specified frequency to avoid spam
        if (this->updateCounter % this->publishFrequency != 0)
        {
            return;
        }

        // Get the pose component of the entity
        auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->entityId);

        if (poseComp)
        {
            // Get the pose data
            auto const &pose = poseComp->Data();

            // Print to terminal
            std::cout << "=== Entity Position Update ===" << std::endl;
            std::cout << "Entity: " << this->entityName << " (ID: " << this->entityId << ")" << std::endl;
            std::cout << "Position - X: " << pose.Pos().X() << ", Y: " << pose.Pos().Y() << ", Z: " << pose.Pos().Z()
                      << std::endl;
            std::cout << "Orientation - Roll: " << pose.Rot().Roll() << ", Pitch: " << pose.Rot().Pitch()
                      << ", Yaw: " << pose.Rot().Yaw() << std::endl;
            std::cout << "Quaternion - W: " << pose.Rot().W() << ", X: " << pose.Rot().X() << ", Y: " << pose.Rot().Y()
                      << ", Z: " << pose.Rot().Z() << std::endl;
            std::cout << "Simulation time: " << _info.simTime.count() / 1e9 << " seconds" << std::endl;
            std::cout << "===============================" << std::endl;

            // Create and populate the pose message for publishing
            gz::msgs::Pose poseMsg;

            // Set position
            poseMsg.mutable_position()->set_x(pose.Pos().X());
            poseMsg.mutable_position()->set_y(pose.Pos().Y());
            poseMsg.mutable_position()->set_z(pose.Pos().Z());

            // Set orientation (quaternion)
            poseMsg.mutable_orientation()->set_w(pose.Rot().W());
            poseMsg.mutable_orientation()->set_x(pose.Rot().X());
            poseMsg.mutable_orientation()->set_y(pose.Rot().Y());
            poseMsg.mutable_orientation()->set_z(pose.Rot().Z());

            // Set timestamp
            auto timestamp = poseMsg.mutable_header()->mutable_stamp();
            timestamp->set_sec(_info.simTime.count() / 1000000000);
            timestamp->set_nsec(_info.simTime.count() % 1000000000);

            // Set frame ID
            poseMsg.mutable_header()->add_data()->set_key("frame_id");
            poseMsg.mutable_header()->mutable_data(0)->add_value(this->entityName);

            // Publish the message
            this->posePub.Publish(poseMsg);
        }
        else
        {
            std::cerr << "Warning: Could not get pose component for entity " << this->entityName
                      << " (ID: " << this->entityId << ")" << std::endl;
        }
    }
}
