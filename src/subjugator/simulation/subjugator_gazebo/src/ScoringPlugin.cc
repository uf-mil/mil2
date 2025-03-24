#include "ScoringPlugin.hh"

#include <cmath>
#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <iostream>

GZ_ADD_PLUGIN(scoring_plugin::ScoringPlugin, gz::sim::System, scoring_plugin::ScoringPlugin::ISystemConfigure,
              scoring_plugin::ScoringPlugin::ISystemPostUpdate)

namespace scoring_plugin
{

void ScoringPlugin::Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                              gz::sim::EntityComponentManager &ecm, gz::sim::EventManager & /*eventMgr*/)
{
    this->subEntity_ = entity;

    // get gate model name
    if (sdf->HasElement("gate_name"))
        this->gateModelName_ = sdf->Get<std::string>("gate_name");
    else
        gzerr << "[ScoringPlugin] Missing <gate_name> parameter in SDF!\n";

    ecm.Each<gz::sim::components::Name>(
        [&](gz::sim::Entity const &_entity, gz::sim::components::Name const *_nameComp)
        {
            if (_nameComp->Data() == this->gateModelName_)
            {
                this->gateEntity_ = _entity;
                gzdbg << "[ScoringPlugin] Found gate entity: " << _entity << std::endl;
            }
            return true;
        });

    if (!rclcpp::ok())
        rclcpp::init(0, nullptr);

    this->rosNode_ = std::make_shared<rclcpp::Node>("scoring_plugin_node");
    this->scorePub_ = this->rosNode_->create_publisher<std_msgs::msg::Int32>("/score", 10);
}

void ScoringPlugin::PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm)
{
    if (info.paused)
        return;

    // Spin the ROS node so we can publish
    rclcpp::spin_some(this->rosNode_);

    if (this->gateEntity_ == gz::sim::kNullEntity)
    {
        // Attempt to find the gate again, if needed
        ecm.Each<gz::sim::components::Name>(
            [&](gz::sim::Entity const &_entity, gz::sim::components::Name const *_nameComp)
            {
                if (_nameComp->Data() == this->gateModelName_)
                {
                    this->gateEntity_ = _entity;
                    gzdbg << "[ScoringPlugin] Found gate entity in PostUpdate: " << _entity << std::endl;
                }
                return true;
            });

        // Gate not found yet
        if (this->gateEntity_ == gz::sim::kNullEntity)
        {
            return;
        }
    }

    // Get positions
    auto subPoseComp = ecm.Component<gz::sim::components::Pose>(this->subEntity_);
    auto gatePoseComp = ecm.Component<gz::sim::components::Pose>(this->gateEntity_);

    // Cannot proceed if either position is missing
    if (!subPoseComp || !gatePoseComp)
    {
        return;
    }
    gz::math::Pose3d subPose = subPoseComp->Data();
    gz::math::Pose3d gatePose = gatePoseComp->Data();

    // Detect pass
    bool passed = this->SubPassedThroughGate(subPose, gatePose);
    if (passed && !this->gatePassed_)
    {
        double DegreesDiff = getAngleBetweenVectors(subPose, gatePose);
        if (DegreesDiff > 5.0)
        {
            this->score_ += 80;
            std::cout << "awarding 80 points: angle penalty" << std::endl;
        }
        else
        {
            this->score_ += 100;
            std::cout << "awarding 100 points: Perfect!" << std::endl;
        }

        std::cout << "Sub passed through gate! Score: " << this->score_ << std::endl;
        this->gatePassed_ = true;

        // Publish score
        std_msgs::msg::Int32 msg;
        msg.data = this->score_;
        this->scorePub_->publish(msg);
    }
}

bool ScoringPlugin::SubPassedThroughGate(gz::math::Pose3d const &subPose, gz::math::Pose3d const &gatePose)
{
    // Very naive example: check distance
    double dist = subPose.Pos().Distance(gatePose.Pos());
    return (dist < 1.0);
}

// For getting angle between sub and gate
double ScoringPlugin::getAngleBetweenVectors(gz::math::Pose3d const &subPose, gz::math::Pose3d const &gatePose)
{
    // getting position relative to world
    gz::math::Vector3d gateForward = gatePose.Rot().RotateVector(gz::math::Vector3d::UnitX);
    gz::math::Vector3d subForward = subPose.Rot().RotateVector(gz::math::Vector3d::UnitX);

    double dot = gateForward.Dot(subForward);
    double mag = gateForward.Length() * subForward.Length();
    if (mag < 1e-6)
        return 0.0;

    double cosAngle = dot / mag;
    if (cosAngle > 1 || cosAngle < -1)
    {
        std::cout << "Possible Rounding Error with cosAngle" << std::endl;
        return 0.0;
    }
    double angleRad = std::acos(cosAngle);
    double angleDeg = angleRad * 180.0 / 3.14159265;
    return angleDeg;
}

}  // namespace scoring_plugin
