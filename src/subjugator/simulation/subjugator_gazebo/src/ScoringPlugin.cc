#include "ScoringPlugin.hh"

#include <cmath>
#include <iostream>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>

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
        std::cout << "[ScoringPlugin] Missing <gate_name> parameter in SDF!\n" << std::endl;

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
    this->scorePub_ = this->rosNode_->create_publisher<std_msgs::msg::Int32>("/score", 5);
    this->eventPub_ = this->rosNode_->create_publisher<std_msgs::msg::String>("/score_events", 5);
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
                    std::cout << "[ScoringPlugin] Found gate entity in PostUpdate: " << _entity << std::endl;
                }
                return true;
            });

        // Gate not found yet
        if (this->gateEntity_ == gz::sim::kNullEntity)
        {
            return;
        }
    }

    auto subPoseComp = ecm.Component<gz::sim::components::Pose>(this->subEntity_);
    auto gatePoseComp = ecm.Component<gz::sim::components::Pose>(this->gateEntity_);

    // Stop if either position is missing
    if (!subPoseComp || !gatePoseComp)
    {
        return;
    }
    gz::math::Pose3d subPose = subPoseComp->Data();
    gz::math::Pose3d gatePose = gatePoseComp->Data();

    double dist = subPose.Pos().Distance(gatePose.Pos());

    if (dist < 5.0 && !inGateStyleZone_ && !gatePassed_)
    {
        // Starting the style zone
        inGateStyleZone_ = true;
        initialPose_ = subPose;

        styleRecord_.lastRoll = subPose.Rot().Roll() * 180.0 / M_PI;
        styleRecord_.lastPitch = subPose.Rot().Pitch() * 180.0 / M_PI;
        styleRecord_.lastYaw = subPose.Rot().Yaw() * 180.0 / M_PI;
        styleRecord_.accumulatedRoll = 0.0;
        styleRecord_.accumulatedPitch = 0.0;
        styleRecord_.accumulatedYaw = 0.0;
        styleRecord_.rollCounter = 0;
        styleRecord_.pitchCounter = 0;
        styleRecord_.yawCounter = 0;

        // Start the style timer
        styleStartTime_ = this->rosNode_->now();
        styleTimerActive_ = true;
        timeLimitReached_ = false;

        std_msgs::msg::String event_msg;
        event_msg.data = "Style zone entered! Rotate and get extra points";
        this->eventPub_->publish(event_msg);
    }
    if (dist > 5.0 && inGateStyleZone_)
    {
        ResetStyleTracking();

        std_msgs::msg::String event_msg;
        event_msg.data = "Style zone left!";
        this->eventPub_->publish(event_msg);
    }

    if (inGateStyleZone_)
    {
        CheckForStylePoints(subPose);

        // Check if style time is up
        if (styleTimerActive_ && (this->rosNode_->now() - styleStartTime_).seconds() > STYLE_TIME_LIMIT)
        {
            styleTimerActive_ = false;
            timeLimitReached_ = true;
            std_msgs::msg::String event_msg;
            event_msg.data = "Style time limit reached!";
            this->eventPub_->publish(event_msg);
        }
    }

    bool passed = this->SubPassedThroughGate(subPose, gatePose);
    if (passed && !this->gatePassed_ && inGateStyleZone_)
    {
        double DegreesDiff = getAngleBetweenVectors(subPose, gatePose);
        int basePoints = 0;
        std::string angleMessage;

        bool hasPerformedStyleManeuvers =
            styleRecord_.rollChanged || styleRecord_.pitchChanged || styleRecord_.yawChanged;

        if (hasPerformedStyleManeuvers || DegreesDiff > 5.0)
        {
            basePoints = 80;
            angleMessage = "angle penalty";
        }
        else
        {
            basePoints = 100;
            angleMessage = "Perfect!";
        }

        // Calculate style points
        int stylePoints = 0;
        if (!timeLimitReached_)
        {
            stylePoints = CalculateStylePoints();
        }
        this->score_ += basePoints + stylePoints;

        std::cout << "Sub passed through gate!" << std::endl;
        std::cout << "Base points: " << basePoints << " (" << angleMessage << ")" << std::endl;
        std::cout << "Style points: " << stylePoints << std::endl;
        std::cout << "Total score: " << this->score_ << std::endl;

        this->gatePassed_ = true;

        // Publish score
        std_msgs::msg::Int32 score_msg;
        score_msg.data = this->score_;
        this->scorePub_->publish(score_msg);

        // Publish event message
        std_msgs::msg::String event_msg;
        event_msg.data = "Gate passed! Base points: " + std::to_string(basePoints) +
                         ", Style points: " + std::to_string(stylePoints) + ", Total: " + std::to_string(this->score_);
        this->eventPub_->publish(event_msg);

        ResetStyleTracking();
    }
}

double ScoringPlugin::angleChange(double previous, double current)
{
    double diff = current - previous;

    // Handle wrap-around for angles
    if (diff > 180.0)
    {
        diff -= 360.0;
    }
    else if (diff < -180.0)
    {
        diff += 360.0;
    }

    return diff;
}

void ScoringPlugin::CheckForStylePoints(gz::math::Pose3d const &currentPose)
{
    double currentRoll = currentPose.Rot().Roll() * 180.0 / M_PI;
    double currentPitch = currentPose.Rot().Pitch() * 180.0 / M_PI;
    double currentYaw = currentPose.Rot().Yaw() * 180.0 / M_PI;

    double rollDiff = angleChange(styleRecord_.lastRoll, currentRoll);
    double pitchDiff = angleChange(styleRecord_.lastPitch, currentPitch);
    double yawDiff = angleChange(styleRecord_.lastYaw, currentYaw);

    // Lambda function
    auto updateAxis = [&](double diff, double &progress, int &quartersAwarded, bool &changedFlag,
                          std::string const &name, int pointsPerQuarter)
    {
        progress += diff;
        int quartersNow = static_cast<int>(std::floor(std::abs(progress) / 90.0));
        if (quartersNow > quartersAwarded)
        {
            changedFlag = true;
            for (int q = quartersAwarded + 1; q <= quartersNow; ++q)
            {
                std_msgs::msg::String m;
                m.data = name + " maneuver detected! +" + std::to_string(pointsPerQuarter) +
                         " style points! (Rotated " + std::to_string(q * 90) + "°)";
                eventPub_->publish(m);
            }
            quartersAwarded = quartersNow;
        }
    };

    // Call the lambda 3 times for roll, pitch, yaw
    updateAxis(rollDiff, styleRecord_.accumulatedRoll, styleRecord_.rollCounter, styleRecord_.rollChanged, "Roll",
               ROLL_POINTS);

    updateAxis(pitchDiff, styleRecord_.accumulatedPitch, styleRecord_.pitchCounter, styleRecord_.pitchChanged, "Pitch",
               PITCH_POINTS);

    updateAxis(yawDiff, styleRecord_.accumulatedYaw, styleRecord_.yawCounter, styleRecord_.yawChanged, "Yaw",
               YAW_POINTS);

    // Store current pose for next time
    styleRecord_.lastRoll = currentRoll;
    styleRecord_.lastPitch = currentPitch;
    styleRecord_.lastYaw = currentYaw;
}

int ScoringPlugin::CalculateStylePoints()
{
    int points = 0;

    if (styleRecord_.rollChanged)
    {
        points += ROLL_POINTS * styleRecord_.rollCounter;
    }

    if (styleRecord_.pitchChanged)
    {
        points += PITCH_POINTS * styleRecord_.pitchCounter;
    }

    if (styleRecord_.yawChanged)
    {
        points += YAW_POINTS * styleRecord_.yawCounter;
    }

    return points;
}

void ScoringPlugin::ResetStyleTracking()
{
    inGateStyleZone_ = false;
    styleTimerActive_ = false;
    styleRecord_ = OrientationRecord();
}

bool ScoringPlugin::SubPassedThroughGate(gz::math::Pose3d const &subPose, gz::math::Pose3d const &gatePose)
{
    gz::math::Pose3d subLocal = gatePose.Inverse() * subPose;

    // Sub's local coordinates in the gate frame
    auto x = subLocal.Pos().X();
    auto y = subLocal.Pos().Y();
    auto z = subLocal.Pos().Z();

    bool inXRange = (x > -1.5 && x < 1.5);
    bool inYRange = (y > -0.2 && y < 0.2);  // thickness of line sub is crossing
    bool inZRange = (z > -1.5 && z < 1.5);  // depth

    return (inXRange && inYRange && inZRange);
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
