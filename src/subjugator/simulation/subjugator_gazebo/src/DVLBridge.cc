/*
 * Copyright (C) 2024 Rakesh Vivekanandan
 * Licensed under the Apache License, Version 2.0
 */

 #include "DVLBridge.hh"

 #include <mutex>
 #include <rclcpp/rclcpp.hpp>
 
 #include <gz/common/Console.hh>
 #include <gz/plugin/Register.hh>
 #include <gz/transport/Node.hh>
 #include <gz/sim/components/Pose.hh>
 
 /// Register the system with Gazebo Sim
 GZ_ADD_PLUGIN(
     dave_ros_gz_plugins::DVLBridge,
     gz::sim::System,
     dave_ros_gz_plugins::DVLBridge::ISystemConfigure,
     dave_ros_gz_plugins::DVLBridge::ISystemPostUpdate)
 
 namespace dave_ros_gz_plugins
 {
 
 struct DVLBridge::PrivateData
 {
   std::mutex mutex;
   gz::transport::Node gz_node;
 
   std::string dvl_topic{"/dvl/velocity"};
 
   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr dvl_pub;
 
   gz::sim::Entity baseEntity{gz::sim::kNullEntity};
 
   gz::math::Pose3d lastBasePose{0, 0, 0, 0, 0, 0};
 };
 

 DVLBridge::DVLBridge()
 : dataPtr(std::make_unique<PrivateData>())
 {}
 

 void DVLBridge::Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &,
                           gz::sim::EventManager &)
 {
   this->dataPtr->baseEntity = _entity;   // Remember the vehicle entity
 
   // Initialise rclcpp once
   if (!rclcpp::ok())
     rclcpp::init(0, nullptr);
 
   auto node = std::make_shared<rclcpp::Node>("dvl_bridge_node");
   this->ros_node_ = node;
 
   if (_sdf->HasElement("topic"))
     this->dataPtr->dvl_topic = _sdf->Get<std::string>("topic");
 
   gzmsg << "[DVLBridge] Gazebo DVL topic: " << this->dataPtr->dvl_topic << '\n';
 
   std::function<void(const gz::msgs::DVLVelocityTracking &)> cb =
       std::bind(&DVLBridge::receiveGazeboCallback, this, std::placeholders::_1);
 
   this->dataPtr->gz_node.Subscribe(this->dataPtr->dvl_topic, std::move(cb));
   this->dataPtr->dvl_pub =
       node->create_publisher<nav_msgs::msg::Odometry>("/dvl/odom", 10);
 }
 
 // Gazebo->ROS callback 
 
 void DVLBridge::receiveGazeboCallback(const gz::msgs::DVLVelocityTracking &msg)
 {
   std::scoped_lock lk(this->dataPtr->mutex);
 
   // Convert world-frame velocity to body frame 
   gz::math::Vector3d v_world(msg.velocity().mean().x(),
                              msg.velocity().mean().y(),
                              msg.velocity().mean().z());
 
   // Rotation WORLD→BODY at this instant (cached each PostUpdate)
   gz::math::Quaterniond q_w2b =
       this->dataPtr->lastBasePose.Rot().Inverse();
 
   gz::math::Vector3d v_body = q_w2b * v_world;
 
   nav_msgs::msg::Odometry odom;
   odom.header.stamp.sec = msg.header().stamp().sec();
   odom.header.stamp.nanosec = msg.header().stamp().nsec();
   odom.header.frame_id = "odom";
   odom.child_frame_id = "base_link";
 
   odom.twist.twist.linear.x = v_body.X();
   odom.twist.twist.linear.y = v_body.Y();
   odom.twist.twist.linear.z = v_body.Z();
 
   odom.pose.pose.orientation.w = 1.0;
 
   constexpr double var = 0.0002;           // DVL does not provide pose
   for (int k = 0; k < 3; ++k)
     odom.twist.covariance[k * 6 + k] = var;
 
   this->dataPtr->dvl_pub->publish(odom);
 }
 

 void DVLBridge::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm)
 {
   if (_info.paused)
     return;
 
   if (auto poseComp =
           _ecm.Component<gz::sim::components::Pose>(this->dataPtr->baseEntity))
   {
     this->dataPtr->lastBasePose = poseComp->Data();
   }
 
   rclcpp::spin_some(this->ros_node_);
 }
 
 }  // namespace dave_ros_gz_plugins
 