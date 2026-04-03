// Copyright 2025 University of Florida Machine Intelligence Laboratory
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "navigator_thruster_mapper/thruster_mapper.h"

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace navigator_thrust_mapper
{

using namespace std::chrono_literals;

class ThrusterMapperNode : public rclcpp::Node
{
  public:
    ThrusterMapperNode() : Node("thrust_mapper")
    {
        this->declare_parameter<std::string>("engine_link_suffix", "_engine_link");
        this->declare_parameter<std::string>("base_link_name", "base_link");
        this->declare_parameter("max_force_pos", 0.0);
        double max_force_pos = this->get_parameter("max_force_pos").as_double();
        this->declare_parameter("max_force_neg", 0.0);
        double max_force_neg = this->get_parameter("max_force_neg").as_double();

        std::string engine_link_suffix = this->get_parameter("engine_link_suffix").as_string();
        std::string base_link_name = this->get_parameter("base_link_name").as_string();
        // Wait for the robot_state_publisher
        auto client = std::make_shared<rclcpp::SyncParametersClient>(this, "/robot_state_publisher");
        while (!client->wait_for_service(100ms))
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for robot_state_publisher");
            if (!rclcpp::ok())
            {
                throw std::runtime_error("Interrupted");
            }
        }

        // Get the robot description
        auto params = client->get_parameters({ "robot_description" });
        if (params.empty())
        {
            throw std::runtime_error("No robot_description found");
        }

        RCLCPP_INFO(this->get_logger(), "Received robot description");
        std::string urdf = params[0].as_string();
        // Parse the urdf and find all engine links
        std::vector<std::string> engine_link_names = find_engine_links(urdf, engine_link_suffix);
        if (engine_link_names.size() == 0)
        {
            std::runtime_error("No engine link found. Exiting");
        }

        // Wait for transforms
        std::vector<std::array<double, 2>> engine_positions;
        std::vector<double> engine_angles;

        tf2_ros::Buffer tf_buffer(this->get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);
        tf2_ros::CreateTimerInterface::SharedPtr cti = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        tf_buffer.setCreateTimerInterface(cti);

        for (auto const& engine_link_name : engine_link_names)
        {
            auto transform = wait_for_transfrom(engine_link_name, base_link_name, tf_buffer);
            engine_positions.emplace_back(
                std::array{ transform.transform.translation.x, transform.transform.translation.y });
            engine_angles.emplace_back(transform.transform.rotation.z);
        }

        // Create publishers
        for (auto const& engine_link_name : engine_link_names)
        {
            std::string engine_name = engine_link_name.substr(0, engine_link_name.size() - engine_link_suffix.size());
            thrust_publishers_.push_back(
                this->create_publisher<std_msgs::msg::Float64>("/thrusters/" + engine_name + "/thrust", 10));
        }

        // Create the thruster map
        std::array<double, 2> force_limit = { max_force_pos, -max_force_neg };
        std::array<double, 2> center_of_mass = { 0.0, 0.0 };
        thruster_map_ = ThrusterMap(engine_link_names, engine_positions, engine_angles, force_limit, center_of_mass);

        // Create a timer to publish thrusts
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33),
                                         std::bind(&ThrusterMapperNode::publish_thrusts, this));

        // Subscribe to the wrench
        wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/wrench/cmd", 1, std::bind(&ThrusterMapperNode::wrench_cb, this, std::placeholders::_1));

        // Subscribe to kill alarm
        kill_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/kill", 1, std::bind(&ThrusterMapperNode::kill_cb, this, std::placeholders::_1));
    }

  private:
    std::vector<std::string> find_engine_links(std::string const& urdf, std::string const& engine_link_suffix)
    {
        if (urdf.empty())
        {
            throw std::runtime_error("Robot description is not set or empty");
        }

        // Parse the urdf
        urdf::Model model;
        if (!model.initString(urdf))
        {
            throw std::runtime_error("Failed to parse URDF. Exiting");
        }

        std::vector<std::string> engine_link_names;
        // Find all engine links
        for (auto const& pair : model.links_)
        {
            std::string const& engine_link_name = pair.first;
            urdf::LinkSharedPtr const& engine_link = pair.second;
            if (!engine_link)
                continue;

            if (pair.first.ends_with(engine_link_suffix))
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "Found engine link " + engine_link_name);
                engine_link_names.push_back(std::move(engine_link_name));
            }
        }

        return engine_link_names;
    }

    geometry_msgs::msg::TransformStamped wait_for_transfrom(std::string const& source, std::string const& target,
                                                            tf2_ros::Buffer& tf_buffer)
    {
        std::string err;
        while (!tf_buffer.canTransform(target, source, tf2::TimePointZero, 100ms, &err) && rclcpp::ok())
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for %s->%s transform to become available", target.c_str(),
                             source.c_str());
        }
        if (!rclcpp::ok())
        {
            throw std::runtime_error("Interrupted");
        }

        RCLCPP_INFO(get_logger(), "Transform %s->%s available", target.c_str(), source.c_str());
        return tf_buffer.lookupTransform(target, source, tf2::TimePointZero);
    }

    void wrench_cb(geometry_msgs::msg::WrenchStamped::SharedPtr const msg)
    {
        wrench_[0] = msg->wrench.force.x;
        wrench_[1] = msg->wrench.force.y;
        wrench_[2] = msg->wrench.torque.z;
    }

    void kill_cb(std_msgs::msg::Bool::SharedPtr const msg)
    {
        kill_ = msg->data;
        if (kill_)
        {
            RCLCPP_WARN(this->get_logger(), "Kill signal received - thrusters disabled");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Kill signal cleared - thrusters enabled");
        }
    }

    void publish_thrusts()
    {
        if (!kill_)
        {
            auto thrusts = thruster_map_.wrench_to_thrusts(wrench_);
            for (size_t i = 0; i < thrusts.size(); ++i)
            {
                std_msgs::msg::Float64 thrust_msg;
                thrust_msg.data = thrusts[i];
                thrust_publishers_[i]->publish(thrust_msg);
            }
        }
        else
        {
            for (size_t i = 0; i < thrust_publishers_.size(); ++i)
            {
                std_msgs::msg::Float64 thrust_msg;
                thrust_msg.data = 0;
                thrust_publishers_[i]->publish(thrust_msg);
            }
        }
    }

    bool kill_{ false };

    std::array<double, 3> wrench_{ { 0.0, 0.0, 0.0 } };
    ThrusterMap thruster_map_;

    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> thrust_publishers_;

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace navigator_thrust_mapper

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<navigator_thrust_mapper::ThrusterMapperNode> node;
    try
    {
        node = std::make_shared<navigator_thrust_mapper::ThrusterMapperNode>();
        rclcpp::spin(node);
    }
    catch (std::exception const& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("thruster_mapper"), "%s. Exiting", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
