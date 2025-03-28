#include "ActiveSonar.hh"
#include <iostream>
#include <gz/transport/Node.hh>

// NOTE: This is needed for GZ_ADD_PLUGIN
#include "gz/plugin/Register.hh"

// Add plugins to gazebo simulator alongside its dependencies
GZ_ADD_PLUGIN(active_sonar::ActiveSonar, active_sonar::ActiveSonar::System, active_sonar::ActiveSonar::ISystemConfigure, active_sonar::ActiveSonar::ISystemPostUpdate)

namespace active_sonar {


  struct ActiveSonar::PrivateData
{
  // Add any private data members here.
  std::mutex scan_mutex_;
  std::mutex echo_mutex_;
  gz::transport::Node gz_node;
  std::string scan_topic;
  std::string echo_topic;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
  rclcpp::Publisher<mil_msgs::msg::EchoIntensities>::SharedPtr echo_pub;
};

ActiveSonar::ActiveSonar() : dataPtr(std::make_unique<PrivateData>()) {}

  
  ActiveSonar::~ActiveSonar()
  {
  }

  void ActiveSonar::Configure(
    const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventManager)
  {


    // Initialize the ROS node and publisher
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    this->ros_node_ = std::make_shared<rclcpp::Node>("active_sonar_node");

    // Declare the topic to subscribe to
    if (!_sdf->HasElement("echo_topic"))
    {
      this->dataPtr->echo_topic = "/lidar/raw_data/points";
      gzmsg << "echo_topic set to default:  " << this->dataPtr->echo_topic << std::endl;
    }
    else
    {
      this->dataPtr->echo_topic = _sdf->Get<std::string>("topic");
      gzmsg << "dvl topic: " << this->dataPtr->echo_topic << std::endl;
    }
    if (!_sdf->HasElement("scan_topic"))
    {
      this->dataPtr->scan_topic = "/lidar/raw_data";
      gzmsg << "echo_topic set to default:  " << this->dataPtr->scan_topic << std::endl;
    }
    else
    {
      this->dataPtr->scan_topic = _sdf->Get<std::string>("topic");
      gzmsg << "dvl topic: " << this->dataPtr->scan_topic << std::endl;
    }


  // Gazebo subscriber
  std::function<void(const gz::msgs::PointCloudPacked &)> echo_callback =
    std::bind(&ActiveSonar::receiveGazeboCallbackEcho, this, std::placeholders::_1);

  std::function<void(const gz::msgs::LaserScan &)> scan_callback =
  std::bind(&ActiveSonar::receiveGazeboCallbackScan, this, std::placeholders::_1);

  this->dataPtr->gz_node.Subscribe(this->dataPtr->echo_topic, echo_callback);
  this->dataPtr->gz_node.Subscribe(this->dataPtr->scan_topic, scan_callback);

  // ROS2 publisher
  this->dataPtr->echo_pub =
    this->ros_node_->create_publisher<mil_msgs::msg::EchoIntensities>(this->dataPtr->echo_topic, 1);
  this->dataPtr->scan_pub =
    this->ros_node_->create_publisher<sensor_msgs::msg::LaserScan>(this->dataPtr->scan_topic, 1);

  }

  void ActiveSonar::receiveGazeboCallbackScan(const gz::msgs::LaserScan & msg)
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->scan_mutex_);
    //std::cout << "SCAN CALLED SUCCESS" << std::endl;

    auto scan_msg = sensor_msgs::msg::LaserScan();

    this->dataPtr->scan_pub->publish(scan_msg);
  }

  void ActiveSonar::receiveGazeboCallbackEcho(const gz::msgs::PointCloudPacked & msg)
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->echo_mutex_);

    gzmsg << "dave_ros_gz_plugins::DVLBridge::receiveGazeboCallback" << std::endl;
    //std::cout << "ECHO CALLED SUCCESS" << std::endl;

    auto echo_msg = mil_msgs::msg::EchoIntensities();

    echo_msg.header.stamp.sec = msg.header().stamp().sec();
    echo_msg.header.stamp.nanosec = msg.header().stamp().nsec();

    //TODO: some of these can be hard coded for now, some from xacro

    // probably hardcode whatever seems reasonable like the ping360
    echo_msg.gain = 0;
    echo_msg.transmit_frequency = 0;
    echo_msg.sound_speed = 0;

    //xacro
    echo_msg.range = 0;
    echo_msg.sample_count = 0;
    echo_msg.angle = 0;

    // repackage data from gz msg
    // sonar_msg.intensities = msg.back().fields(3).name();

    // 
    this->dataPtr->echo_pub->publish(echo_msg);

  }
  
  void ActiveSonar::PostUpdate(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm)
  {
    // Only runs code if the simulation is active
    if (!_info.paused)
    {
      rclcpp::spin_some(this->ros_node_);
      // Create a basic test message to publish to the stored publisher
      //gz::msgs::Twist message;
      //message.mutable_linear()->set_x(1.0);  // Temporary, testing data
      //message.mutable_angular()->set_z(0.5);  //  Temporary, testign data

      // Publish the newly created message to the stored publisher!
      //publisher.Publish(message);
    }
  }
} 