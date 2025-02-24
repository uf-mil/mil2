#include "DepthSensor.hh"
#include <gz/common/Console.hh>
#include <rclcpp/rclcpp.hpp>
#include <gz/plugin/Register.hh>  // For GZ_ADD_PLUGIN
#include <iostream>
#include <random>  


// Register plugin so Gazebo can see it
GZ_ADD_PLUGIN(depth_sensor::DepthSensor, gz::sim::System, depth_sensor::DepthSensor::ISystemConfigure, depth_sensor::DepthSensor::ISystemPostUpdate)

namespace depth_sensor
{

DepthSensor::DepthSensor()
{
}

void DepthSensor::Configure(gz::sim::Entity const &_entity, std::shared_ptr<sdf::Element const> const &_sdf,
    gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventManager)
{   

    // std::cout << "[DepthSensor] Attached to entity ID:" << _entity << std::endl;
    this->modelEntity_ = _entity;

    // Getting values from SDF
    if (_sdf->HasElement("frame_id"))
        this->frameName_ = _sdf->Get<std::string>("frame_id");

    if (_sdf->HasElement("offset"))
        this->offset_ = _sdf->Get<gz::math::Pose3d>("offset");

    if (_sdf->HasElement("update_rate"))
    {
        float rate = _sdf->Get<float>("update_rate");
        if (rate > 0.0){
            this->updatePeriod_ = 1.0 / rate;
        }
    }
    if (_sdf->HasElement("noise_mean"))
        this->noiseMean_ = _sdf->Get<double>("noise_mean");

    if (_sdf->HasElement("noise_stddev"))
        this->noiseStdDev_ = _sdf->Get<double>("noise_stddev");


    // Initialize random engine and distribution
    this->randomEngine_.seed(std::random_device{}());
    this->noiseDist_ = std::normal_distribution<double>(this->noiseMean_, this->noiseStdDev_);

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    this->ros_node_ = std::make_shared<rclcpp::Node>("depth_sensor_node");
    this->depthPub_ = this->ros_node_->create_publisher<mil_msgs::msg::DepthStamped>("/depth", 10);
}


void DepthSensor::PostUpdate(gz::sim::UpdateInfo const &_info, gz::sim::EntityComponentManager const &_ecm)
{
    if (!_info.paused)
    {           
        rclcpp::spin_some(this->ros_node_);
  
        if (_info.iterations % 1000 == 0)
        {
            gzmsg << "depth_sensor::DepthSensor::PostUpdate" << std::endl;
        }

        // Convert simTime (steady_clock::duration) to float (seconds)
        float nowSec = std::chrono::duration_cast<std::chrono::duration<float>>(_info.simTime).count();

        float dt = nowSec - this->lastPubTime_;
        if (dt < this->updatePeriod_)
            return;


        this->lastPubTime_ = nowSec;

        //Get Pose component
        auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->modelEntity_);
        if (!poseComp)
        {
        gzerr << "[DepthSensor] Pose component not found for entity [" 
                << this->modelEntity_ << "]\n";
        return;
        }
        

        // To avoid the deprecated operator+, we manually combine:
        gz::math::Pose3d finalPose = poseComp->Data();
        finalPose.Pos() += this->offset_ .Pos();
        finalPose.Rot() = finalPose.Rot() * this->offset_ .Rot();

        // Depth is neg Z
        float depth = -finalPose.Pos().Z();
        if (depth < 0.0){
            depth = 0.0;
        }

        if (this->noiseStdDev_ > 0.0){
            double noise = this->noiseDist_(this->randomEngine_);
            
            // This keeps the std the same, avoids "clamping"
            // Does not "Clamp" values given by distribution
            if (depth < std::abs(noise)){
                depth = 0.0;
            }
            else{
                depth += static_cast<float>(noise);
            }
        } 
            
        //64 bit int for large Nanosecond Values
        int64_t simTimeNS = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();
        int32_t secPart  = static_cast<int32_t>(simTimeNS / 1000000000);
        uint32_t nsecPart = static_cast<uint32_t>(simTimeNS % 1000000000);

        rclcpp::Time simTime(secPart, nsecPart, RCL_SYSTEM_TIME);
        

        // Publish
        mil_msgs::msg::DepthStamped msg;
        msg.header.frame_id = this->frameName_;
        msg.header.stamp = simTime;
        msg.depth = depth;

        this->depthPub_->publish(msg);
    }
}
} // namespace