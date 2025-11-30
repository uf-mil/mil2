#include "GripperControl.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

namespace gripper_control
{

GripperControl::GripperControl()
{
}

GripperControl::~GripperControl()
{
}

void GripperControl::Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                               gz::sim::EntityComponentManager &ecm, gz::sim::EventManager & /*eventMgr*/)
{
    std::cout << "[GripperControl] Configuring..." << std::endl;

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
    this->node_ = std::make_shared<rclcpp::Node>("gripper_control_plugin_node");

    // Subscribe to keyboard presses (std_msgs::msg::String)
    this->key_sub_ = this->node_->create_subscription<std_msgs::msg::String>(
        "/keyboard/keypress", 10, std::bind(&GripperControl::KeypressCallback, this, std::placeholders::_1));

    auto nameComp = ecm.Component<gz::sim::components::Name>(entity);
    if (nameComp)
    {
        this->model_name_ = nameComp->Data();
        std::cout << "[GripperControl] Got model name from Entity: " << this->model_name_ << std::endl;
    }
    else if (sdf && sdf->HasAttribute("name"))
    {
        try
        {
            this->model_name_ = sdf->Get<std::string>("name");
            std::cout << "[GripperControl] Got model name from SDF: " << this->model_name_ << std::endl;
        }
        catch (...)
        {
            gzerr << "[GripperControl] Failed to get model name from SDF attribute 'name'." << std::endl;
        }
    }
    else
    {
        gzerr << "[GripperControl] WARNING: Could not determine model name; plugin topic may be invalid." << std::endl;
    }

    if (sdf)
    {
        if (sdf->HasElement("joint_name"))
        {
            this->joint_name_ = sdf->Get<std::string>("joint_name");
        }
        if (sdf->HasElement("open_pos"))
        {
            this->open_pos_ = sdf->Get<double>("open_pos");
        }
        if (sdf->HasElement("closed_pos"))
        {
            this->closed_pos_ = sdf->Get<double>("closed_pos");
        }
    }

    // Build Gazebo joint command topic
    if (!this->model_name_.empty())
    {
        // topic pattern: /model/<model_name>/joint/<joint_name>/cmd_pos
        this->topic_name_ = "/model/" + this->model_name_ + "/joint/" + this->joint_name_ + "/cmd_pos";
        std::cout << "[GripperControl] Advertising gz topic: " << this->topic_name_ << std::endl;
        this->gz_pub_ = this->gz_node_.Advertise<gz::msgs::Double>(this->topic_name_);
    }
}

void GripperControl::KeypressCallback(std_msgs::msg::String::SharedPtr msg)
{
    if (!msg)
        return;

    if (msg->data == "u" && !this->u_pressed_)
    {
        this->u_pressed_ = true;
        std::cout << "[GripperControl] 'u' pressed (queued toggle)" << std::endl;
    }
}

void GripperControl::PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const & /*ecm*/)
{
    if (info.paused)
        return;

    rclcpp::spin_some(this->node_);

    // If a 'u' press was detected, toggle and publish the new joint command
    if (this->u_pressed_)
    {
        this->gripper_open_ = !this->gripper_open_;
        double cmd = this->gripper_open_ ? this->open_pos_ : this->closed_pos_;

        if (!this->topic_name_.empty())
        {
            gz::msgs::Double msg;
            msg.set_data(cmd);
            this->gz_pub_.Publish(msg);
            std::cout << "[GripperControl] Published cmd_pos = " << cmd << " to " << this->topic_name_
                      << " (gripper_open=" << (this->gripper_open_ ? "true" : "false") << ")" << std::endl;
        }
        else
        {
            gzerr << "[GripperControl] Cannot publish - topic name empty." << std::endl;
        }

        // Reset one-shot flag
        this->u_pressed_ = false;
    }
}

}  // namespace gripper_control

GZ_ADD_PLUGIN(gripper_control::GripperControl, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
