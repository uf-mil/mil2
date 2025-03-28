#include "ActiveSonar.hh"
#include <iostream>

namespace active_sonar {
  ActiveSonar::ActiveSonar()
  {
  }
  
  ActiveSonar::~ActiveSonar()
  {
  }

  void ActiveSonar::Configure(
    const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventManager)
  {
    // // Declare the topic to subscribe to
    // std::string topic_sub = "active_sonar";

    // //
    // // CHANGE TO BE ACTUAL DATA (Temporary callback for testing purposes)
    // // 
    // std::function<void(const gz::msgs::Twist &)> callback;
    
    // // Output an error if failure to subscribe to the active_sonar topic
    // if (!this->node.Subscribe(topic_sub, callback))
    // {
    //   std::cerr << "Error subscribing to topic for active_sonar." << std::endl;
    // }

    // // Declare the topic to publish to and register the topic with the transport system
    // std::string publish_topic = "/active_sonar/raw_data";
    // this->publisher = node.Advertise<gz::msgs::Twist>(publish_topic);
  }

  void ActiveSonar::receiveGazeboCallback(const gz::msgs::PointCloudPacked & msg)
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex_);

    gzmsg << "dave_ros_gz_plugins::DVLBridge::receiveGazeboCallback" << std::endl;

    auto sonar_msg = mil_msgs::msg::EchoIntensities();

    sonar_msg.header.stamp.sec = msg.header().stamp().sec();
    sonar_msg.header.stamp.nanosec = msg.header().stamp().nsec();

    //TODO: some of these can be hard coded for now, some from xacro

    // probably hardcode whatever seems reasonable like the ping360
    sonar_msg.gain = 0;
    sonar_msg.transmit_frequency = 0;
    sonar_msg.sound_speed = 0;

    //xacro
    sonar_msg.range = 0;
    sonar_msg.sample_count = 0;
    sonar_msg.angle = 0;

    // repackage data from gz msg
    sonar_msg.intensities = msg.back().fields(3).name();

    //todo: create sonar publisher
    this->publisher->publish(sonar_msg);

  }
  
  void ActiveSonar::PostUpdate(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm)
  {
    // Only runs code if the simulation is active
    if (!_info.paused)
    {
      // Create a basic test message to publish to the stored publisher
      //gz::msgs::Twist message;
      //message.mutable_linear()->set_x(1.0);  // Temporary, testing data
      //message.mutable_angular()->set_z(0.5);  //  Temporary, testign data

      // Publish the newly created message to the stored publisher!
      //publisher.Publish(message);
    }
  }
}

// Add plugins to gazebo simulator alongside its dependencies
GZ_ADD_PLUGIN(
    active_sonar::ActiveSonar,
    gz::sim::System,
    active_sonar::ActiveSonar::ISystemConfigure,
    active_sonar::ActiveSonar::ISystemPostUpdate
)