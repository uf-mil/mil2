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
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "navigator_thrust_mapper/thruster_map.h"

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

using std::placeholders::_1;

namespace navigator_thrust_mapper
{

class ThrusterMapperNode : public rclcpp::Node
{
  public:
    ThrusterMapperNode() : Node("thrust_mapper")
    {
        this->declare_parameter<bool>("is_vrx", false);
        this->declare_parameter<bool>("is_simulation", false);
        this->declare_parameter<std::string>("robot_description", "");

        is_vrx_ = this->get_parameter("is_vrx").as_bool();
        is_sim_ = this->get_parameter("is_simulation").as_bool();

        std::string urdf = this->get_parameter("robot_description").as_string();
        if (urdf.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "robot description not set or empty");
            throw std::runtime_error("robot description not set or empty");
        }

        // create thruster_map from URDF
        if (is_vrx_ || is_sim_)
        {
            thruster_map_ = ThrusterMap::from_vrx_urdf(urdf);
        }
        else
        {
            thruster_map_ = ThrusterMap::from_urdf(urdf);
        }

        thruster_names_ = thruster_map_.names;
        thrust_string_index_ = (is_vrx_ ? 5 : 0);

        if (is_vrx_ || is_sim_)
        {
            for (auto const &name : thruster_names_)
            {
                std::string topic = "/wamv/thrusters/" + name.substr(thrust_string_index_) + "_thrust_cmd";
                publishers_float_.push_back(this->create_publisher<std_msgs::msg::Float32>(topic, 1));
            }
        }
        else
        {
            for (auto const &name : thruster_names_)
            {
                std::string topic = "/" + name + "_motor/cmd";
                publishers_float_.push_back(this->create_publisher<std_msgs::msg::Float32>(topic, 1));
            }
        }

        if (!is_vrx_ && !is_sim_)
        {
            joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/thruster_states", 1);
            joint_state_msg_ = sensor_msgs::msg::JointState();
            for (auto const &j : thruster_map_.joints)
            {
                joint_state_msg_.name.push_back(j);
                joint_state_msg_.position.push_back(0.0);
                joint_state_msg_.effort.push_back(0.0);
            }
        }

        wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/wrench/cmd", 1, std::bind(&ThrusterMapperNode::wrench_cb, this, _1));

        // Subscribe to kill alarm
        kill_sub_ = this->create_subscription<std_msgs::msg::Bool>("/kill", 1,
                                                                   std::bind(&ThrusterMapperNode::kill_cb, this, _1));

        // timer to publish at 30 Hz
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33),
                                         std::bind(&ThrusterMapperNode::publish_thrusts, this));

        RCLCPP_INFO(this->get_logger(), "ThrusterMapperNode initialized with %zu thrusters", thruster_names_.size());
    }

  private:
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
        std::vector<float> commands(publishers_float_.size(), 0.0f);

        if (!kill_)
        {
            auto thrusts_d = thruster_map_.wrench_to_thrusts(wrench_);
            if (thrusts_d.size() != publishers_float_.size())
            {
                RCLCPP_FATAL(this->get_logger(), "Number of thrusts does not equal number of publishers");
                return;
            }
            for (size_t i = 0; i < thrusts_d.size(); ++i)
            {
                commands[i] = static_cast<float>(thrusts_d[i]);
            }
        }

        if (!is_vrx_ && !is_sim_)
        {
            for (size_t i = 0; i < publishers_float_.size(); ++i)
            {
                joint_state_msg_.effort[i] = commands[i];
                std_msgs::msg::Float32 m;
                m.data = commands[i];
                publishers_float_[i]->publish(m);
            }
            joint_state_pub_->publish(joint_state_msg_);
        }
        else
        {
            for (size_t i = 0; i < publishers_float_.size(); ++i)
            {
                std_msgs::msg::Float32 m;
                m.data = commands[i];
                publishers_float_[i]->publish(m);
            }
        }
    }

    bool is_vrx_{ false };
    bool is_sim_{ false };
    bool kill_{ false };
    size_t thrust_string_index_{ 0 };

    std::array<double, 3> wrench_{ { 0.0, 0.0, 0.0 } };
    std::vector<std::string> thruster_names_;

    ThrusterMap thruster_map_;

    std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers_float_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    sensor_msgs::msg::JointState joint_state_msg_;

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace navigator_thrust_mapper

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<navigator_thrust_mapper::ThrusterMapperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
