#include "ActiveSonar.hh"

#include <gz/transport/Node.hh>
#include <iostream>

// NOTE: This is needed for GZ_ADD_PLUGIN
#include "gz/plugin/Register.hh"

// Add plugins to gazebo simulator alongside its dependencies
GZ_ADD_PLUGIN(active_sonar::ActiveSonar, active_sonar::ActiveSonar::System, active_sonar::ActiveSonar::ISystemConfigure,
              active_sonar::ActiveSonar::ISystemPostUpdate)

namespace active_sonar
{

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

ActiveSonar::ActiveSonar() : dataPtr(std::make_unique<PrivateData>())
{
}

ActiveSonar::~ActiveSonar()
{
}

void ActiveSonar::Configure(gz::sim::Entity const &_entity, std::shared_ptr<sdf::Element const> const &_sdf,
                            gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventManager)
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
    this->dataPtr->echo_topic = "/active_sonar/raw_data/points";
    gzmsg << "echo_topic set to default:  " << this->dataPtr->echo_topic << std::endl;
  }
  else
  {
    this->dataPtr->echo_topic = _sdf->Get<std::string>("topic");
    gzmsg << "dvl topic: " << this->dataPtr->echo_topic << std::endl;
  }
  if (!_sdf->HasElement("scan_topic"))
  {
    this->dataPtr->scan_topic = "/active_sonar/raw_data";
    gzmsg << "echo_topic set to default:  " << this->dataPtr->scan_topic << std::endl;
  }
  else
  {
    this->dataPtr->scan_topic = _sdf->Get<std::string>("topic");
    gzmsg << "dvl topic: " << this->dataPtr->scan_topic << std::endl;
  }

  // Gazebo subscriber
  std::function<void(gz::msgs::PointCloudPacked const &)> echo_callback =
      std::bind(&ActiveSonar::receiveGazeboCallbackEcho, this, std::placeholders::_1);

  std::function<void(gz::msgs::LaserScan const &)> scan_callback =
      std::bind(&ActiveSonar::receiveGazeboCallbackScan, this, std::placeholders::_1);

  this->dataPtr->gz_node.Subscribe(this->dataPtr->echo_topic, echo_callback);
  this->dataPtr->gz_node.Subscribe(this->dataPtr->scan_topic, scan_callback);

  // ROS2 publisher
  this->dataPtr->echo_pub =
      this->ros_node_->create_publisher<mil_msgs::msg::EchoIntensities>(this->dataPtr->echo_topic, 1);
  this->dataPtr->scan_pub =
      this->ros_node_->create_publisher<sensor_msgs::msg::LaserScan>(this->dataPtr->scan_topic, 1);
}

// Serves as the callback for the LaserScan
void ActiveSonar::receiveGazeboCallbackScan(gz::msgs::LaserScan const &msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->scan_mutex_);

  auto scan_msg = sensor_msgs::msg::LaserScan();

  // Get the gazebo msg info and store into ros2 msg values
  // Header
  gz::msgs::Header const &header = msg.header();
  scan_msg.header.stamp.sec = header.stamp().sec();
  scan_msg.header.stamp.nanosec = header.stamp().nsec();
  scan_msg.header.frame_id = "sonar_frame";

  // Angle info and convert to proper variable types
  scan_msg.angle_min = static_cast<float>(msg.angle_min());
  scan_msg.angle_max = static_cast<float>(msg.angle_max());
  scan_msg.angle_increment = static_cast<float>(msg.angle_step());
  // Time info and scan are set at 0 since the time between
  // measurements and scans is irrelevant for use right now
  scan_msg.time_increment = 0;
  scan_msg.scan_time = 0;

  // Range info
  scan_msg.range_min = static_cast<float>(msg.range_min());
  scan_msg.range_max = static_cast<float>(msg.range_max());

  // Fill in the ranges array
  int num_ranges = msg.ranges_size();
  scan_msg.ranges.resize(num_ranges);
  for (int i = 0; i < num_ranges; i++)
  {
    // If value < range_min or value > range_max, discard it
    double rangeValue = msg.ranges(i);
    if (rangeValue > msg.range_min() && rangeValue < msg.range_max())
    {
      scan_msg.ranges[i] = static_cast<float>(rangeValue);
    }
  }

  // Fill in the intensities array
  int num_intensities = msg.intensities_size();
  scan_msg.intensities.resize(num_intensities);
  for (int i = 0; i < num_intensities; i++)
  {
    // Typecast to float
    double intensityValue = msg.intensities(i);
    scan_msg.intensities[i] = static_cast<float>(intensityValue);
  }

  // Publish ros msg
  this->dataPtr->scan_pub->publish(scan_msg);
}

// Serves as the callback for the PointCloudPacked data
void ActiveSonar::receiveGazeboCallbackEcho(gz::msgs::PointCloudPacked const &msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->echo_mutex_);

  gzmsg << "dave_ros_gz_plugins::DVLBridge::receiveGazeboCallback" << std::endl;

  auto echo_msg = mil_msgs::msg::EchoIntensities();

  echo_msg.header.stamp.sec = msg.header().stamp().sec();
  echo_msg.header.stamp.nanosec = msg.header().stamp().nsec();

  //
  // TODO: Get proper data for these hardcoded messages
  //
  echo_msg.gain = 0;
  echo_msg.transmit_frequency = 0;
  echo_msg.sound_speed = 0;

  //
  // TODO: Grab data for these attributes from the active_sonar.xacro (Do not hardcode, import them if possible)
  //
  echo_msg.range = 0;
  echo_msg.sample_count = 0;
  echo_msg.angle = 0;

  // Repackage intensity data from Gazebo to ROS2
  // echo_msg.intensities = msg.intensities();
  // std::cout << msg.data() << std::endl;
  // 
  // this->dataPtr->echo_pub->publish(echo_msg);
}

void ActiveSonar::PostUpdate(gz::sim::UpdateInfo const &_info, gz::sim::EntityComponentManager const &_ecm)
{
  // Only runs code if the simulation is active
  if (!_info.paused)
  {
    rclcpp::spin_some(this->ros_node_);
    // Create a basic test message to publish to the stored publisher
    // gz::msgs::Twist message;
    // message.mutable_linear()->set_x(1.0);  // Temporary, testing data
    // message.mutable_angular()->set_z(0.5);  //  Temporary, testign data

    // Publish the newly created message to the stored publisher!
    // publisher.Publish(message);
  }
}
}  // namespace active_sonar
