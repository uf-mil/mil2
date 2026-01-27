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
    std::cout << "[GripperControl] Configuring GripperControl Plugin ..." << std::endl;

    // Initialize ROS2 if not already running
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    // Create a ROS2 node for this plugin and subscribe to keyboard keypresses
    this->node_ = std::make_shared<rclcpp::Node>("gripper_control_plugin_node");
    this->key_sub_ = this->node_->create_subscription<std_msgs::msg::String>(
        "/keyboard/keypress", 10, std::bind(&GripperControl::KeypressCallback, this, std::placeholders::_1));

    std::cout << "[GripperControl] ROS2 node created and subscribed to /keyboard/keypress" << std::endl;

    // Try to obtain model name (from Entity Name component or SDF attribute)
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
        catch (std::exception const &e)
        {
            gzerr << "[GripperControl] Failed to get model name from SDF attribute 'name': " << e.what() << std::endl;
        }
    }
    else
    {
        gzerr << "[GripperControl] WARNING: Could not determine model name; plugin topic may be invalid." << std::endl;
    }

    // Parse SDF parameters for joint name and positions (if provided)
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

    // Determine world name (for logging) by iterating Worlds
    ecm.Each<gz::sim::components::World, gz::sim::components::Name>(
        [&](gz::sim::Entity const & /*ent*/, gz::sim::components::World const *,
            gz::sim::components::Name const *name) -> bool
        {
            this->world_name_ = name->Data();
            return false;  // stop after first world found
        });

    std::cout << "[GripperControl] World Name: " << this->world_name_ << std::endl;

    // Build Gazebo joint command topic and advertise
    if (!this->model_name_.empty())
    {
        // topic pattern: /model/<model_name>/joint/<joint_name>/cmd_pos
        this->topic_name_ = "/model/" + this->model_name_ + "/joint/" + this->joint_name_ + "/cmd_pos";
        std::cout << "[GripperControl] Advertising gz topic: " << this->topic_name_ << std::endl;
        this->gz_pub_ = this->gz_node_.Advertise<gz::msgs::Double>(this->topic_name_);
        // Small sanity log if advertise failed to produce a valid publisher (depends on gz implementation)
        if (!this->gz_pub_)  // operator bool available in some Gazebo versions
        {
            gzerr << "[GripperControl] Failed to create gz publisher for topic: " << this->topic_name_ << std::endl;
        }
    }
    else
    {
        gzerr << "[GripperControl] Topic name left empty because model name is unknown." << std::endl;
    }
}

void GripperControl::KeypressCallback(std_msgs::msg::String::SharedPtr const msg)
{
    if (!msg)
        return;

    std::cout << "[GripperControl] Keypress received: " << msg->data << std::endl;

    // Toggle only on 'u' press (one-shot)
    if (msg->data == "u" && !this->u_pressed_)
    {
        this->u_pressed_ = true;
    }
}

void GripperControl::PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const & /*ecm*/)
{
    if (info.paused)
        return;

    // Let ROS2 handle incoming messages for this node
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

// Register plugin for Gazebo
GZ_ADD_PLUGIN(gripper_control::GripperControl, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPostUpdate)
