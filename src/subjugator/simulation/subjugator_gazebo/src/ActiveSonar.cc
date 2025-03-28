#include "ActiveSonar.hh"
#include <iostream>

// NOTE: This is needed for GZ_ADD_PLUGIN
#include "gz/plugin/Register.hh"

// Add plugins to gazebo simulator alongside its dependencies
GZ_ADD_PLUGIN(active_sonar::ActiveSonar, active_sonar::ActiveSonar::System, active_sonar::ActiveSonar::ISystemConfigure, active_sonar::ActiveSonar::ISystemPostUpdate)

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
    // Declare the topic to subscribe to
    std::string topic_sub = "active_sonar";

    // Initialize the ROS node and publisher
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    // Create a ROS2 node for publishing
    this->node = std::make_shared<rclcpp::Node>("/active_sonar/echo_intensities");
    this->publisher = this->node->create_publisher<mil_msgs::msg::EchoIntensities>("/echo", 1);
  }

  void ActiveSonar::receiveGazeboCallback(const gz::msgs::PointCloudPacked & msg)
  {
    // std::lock_guard<std::mutex> lock(this->dataPtr->mutex_);

    gzmsg << "dave_ros_gz_plugins::DVLBridge::receiveGazeboCallback" << std::endl;

    auto sonar_msg = mil_msgs::msg::EchoIntensities();

    // sonar_msg.header.stamp.sec = msg.header().stamp().sec();
    // sonar_msg.header.stamp.nanosec = msg.header().stamp().nsec();

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
    // sonar_msg.intensities = msg.back().fields(3).name();

    // 
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