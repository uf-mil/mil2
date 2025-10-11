#include "simulated_kill_board.h"

#include <algorithm>
#include <iostream>

#include "constants.h"

namespace navigator_kill_board
{

// =============================================================================
//                                NoopSerial
// =============================================================================

NoopSerial::NoopSerial()
{
    // Intentionally empty - no operation
}

NoopSerial::~NoopSerial()
{
    // Intentionally empty - no operation
}

void NoopSerial::open()
{
    // No operation
}

int NoopSerial::in_waiting() const
{
    return 0;
}

int NoopSerial::out_waiting() const
{
    return 0;
}

void NoopSerial::close()
{
    // No operation
}

std::string NoopSerial::read(int length)
{
    (void)length;  // Suppress unused parameter warning
    return "";
}

int NoopSerial::write(std::string const& data)
{
    return static_cast<int>(data.length());
}

void NoopSerial::flush()
{
    // No operation
}

void NoopSerial::flushInput()
{
    // No operation
}

void NoopSerial::flushOutput()
{
    // No operation
}

void NoopSerial::reset_input_buffer()
{
    // No operation
}

void NoopSerial::reset_output_buffer()
{
    // No operation
}

void NoopSerial::send_break()
{
    // No operation
}

// =============================================================================
//                              SimulatedSerial
// =============================================================================

SimulatedSerial::SimulatedSerial() : buffer_("")
{
}

int SimulatedSerial::in_waiting() const
{
    return static_cast<int>(buffer_.length());
}

void SimulatedSerial::reset_input_buffer()
{
    buffer_.clear();
}

std::string SimulatedSerial::read(int length)
{
    // Handle case where buffer is smaller than requested length
    int actual_length = std::min(length, static_cast<int>(buffer_.size()));

    std::string result = buffer_.substr(0, actual_length);
    buffer_ = buffer_.substr(actual_length);
    return result;
}

// =============================================================================
//                            SimulatedKillBoard
// =============================================================================

SimulatedKillBoard::SimulatedKillBoard() : has_last_ping_(false), killed_(false), light_("OFF")
{
    node_ = rclcpp::Node::make_shared("simulated_kill_board");

    // Initialize memory map - all kill sources start as false
    memory_["BUTTON_FRONT_PORT"] = false;
    memory_["BUTTON_AFT_PORT"] = false;
    memory_["BUTTON_FRONT_STARBOARD"] = false;
    memory_["BUTTON_AFT_STARBOARD"] = false;
    memory_["COMPUTER"] = false;
    memory_["HEARTBEAT_COMPUTER"] = false;
    memory_["BUTTON_REMOTE"] = false;
    memory_["HEARTBEAT_REMOTE"] = false;

    // Create ROS services for button kill sources
    for (auto const& kill_name : KILLS)
    {
        if (kill_name.find("BUTTON") == 0)
        {
            auto service = node_->create_service<std_srvs::srv::SetBool>(
                kill_name, [this, kill_name](std::shared_ptr<std_srvs::srv::SetBool::Request> const request,
                                             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
                { this->set_button_callback(kill_name, request, response); });
            service_servers_.push_back(service);
        }
    }

    // Create timer for periodic checks (200ms period)
    timer_ =
        node_->create_wall_timer(std::chrono::milliseconds(200), std::bind(&SimulatedKillBoard::timer_callback, this));
}

void SimulatedKillBoard::set_button_callback(std::string const& button,
                                             std::shared_ptr<std_srvs::srv::SetBool::Request> const request,
                                             std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    set_kill(button, request->data);
    response->success = true;
}

void SimulatedKillBoard::timer_callback()
{
    check_timeout();
}

void SimulatedKillBoard::check_timeout()
{
    if (!has_last_ping_)
    {
        return;
    }

    double time_diff = (node_->now() - last_ping_).seconds();
    if (time_diff >= TIMEOUT_SECONDS)
    {
        set_kill("HEARTBEAT_COMPUTER", true);
    }
    else
    {
        set_kill("HEARTBEAT_COMPUTER", false);
    }
}

void SimulatedKillBoard::set_kill(std::string const& name, bool on, bool update)
{
    if (memory_[name] != on)
    {
        memory_[name] = on;

        // Update overall killed status - equivalent to np.any()
        killed_ = false;
        for (auto const& pair : memory_)
        {
            if (pair.second)
            {
                killed_ = true;
                break;
            }
        }
    }

    if (!update)
    {
        return;
    }

    // Add appropriate response to buffer based on the kill state
    // Buffer initially empty
    // Buffer += KILL_SOURCE_TRUE if Button is pressed (on=true), else KILL_SOURCE_FALSE
    if (name == "BUTTON_FRONT_PORT")
    {
        buffer_ += on ? BUTTON_FRONT_PORT_TRUE : BUTTON_FRONT_PORT_FALSE;
    }
    else if (name == "BUTTON_AFT_PORT")
    {
        buffer_ += on ? BUTTON_AFT_PORT_TRUE : BUTTON_AFT_PORT_FALSE;
    }
    else if (name == "BUTTON_FRONT_STARBOARD")
    {
        buffer_ += on ? BUTTON_FRONT_STARBOARD_TRUE : BUTTON_FRONT_STARBOARD_FALSE;
    }
    else if (name == "BUTTON_AFT_STARBOARD")
    {
        buffer_ += on ? BUTTON_AFT_STARBOARD_TRUE : BUTTON_AFT_STARBOARD_FALSE;
    }
    else if (name == "COMPUTER")
    {
        buffer_ += on ? COMPUTER_TRUE : COMPUTER_FALSE;
    }
    else if (name == "HEARTBEAT_COMPUTER")
    {
        buffer_ += on ? HEARTBEAT_COMPUTER_TRUE : HEARTBEAT_COMPUTER_FALSE;
    }
    else if (name == "BUTTON_REMOTE")
    {
        buffer_ += on ? BUTTON_REMOTE_TRUE : BUTTON_REMOTE_FALSE;
    }
    else if (name == "HEARTBEAT_REMOTE")
    {
        buffer_ += on ? HEARTBEAT_REMOTE_TRUE : HEARTBEAT_REMOTE_FALSE;
    }

    // Add overall status
    buffer_ += killed_ ? OVERALL_TRUE : OVERALL_FALSE;
}

void SimulatedKillBoard::set_light(std::string const& status)
{
    if (light_ != status)
    {
        std::cout << "setting lights " << status << std::endl;
        light_ = status;
    }
}

char SimulatedKillBoard::get_response(bool boolean) const
{
    return boolean ? RESPONSE_TRUE : RESPONSE_FALSE;
}

void SimulatedKillBoard::get_status(char byte)
{
    if (byte == OVERALL_REQUEST)
    {
        buffer_ = get_response(killed_) + buffer_;
        return;
    }

    // Check each memory item for status requests
    if (byte == BUTTON_FRONT_PORT_REQUEST)
    {
        buffer_ = get_response(memory_.at("BUTTON_FRONT_PORT")) + buffer_;
    }
    else if (byte == BUTTON_AFT_PORT_REQUEST)
    {
        buffer_ = get_response(memory_.at("BUTTON_AFT_PORT")) + buffer_;
    }
    else if (byte == BUTTON_FRONT_STARBOARD_REQUEST)
    {
        buffer_ = get_response(memory_.at("BUTTON_FRONT_STARBOARD")) + buffer_;
    }
    else if (byte == BUTTON_AFT_STARBOARD_REQUEST)
    {
        buffer_ = get_response(memory_.at("BUTTON_AFT_STARBOARD")) + buffer_;
    }
    else if (byte == COMPUTER_REQUEST)
    {
        buffer_ = get_response(memory_.at("COMPUTER")) + buffer_;
    }
    else if (byte == HEARTBEAT_COMPUTER_REQUEST)
    {
        buffer_ = get_response(memory_.at("HEARTBEAT_COMPUTER")) + buffer_;
    }
    else if (byte == BUTTON_REMOTE_REQUEST)
    {
        buffer_ = get_response(memory_.at("BUTTON_REMOTE")) + buffer_;
    }
    else if (byte == HEARTBEAT_REMOTE_REQUEST)
    {
        buffer_ = get_response(memory_.at("HEARTBEAT_REMOTE")) + buffer_;
    }
}

void SimulatedKillBoard::handle_sync(char data)
{
    if (data == PING_REQUEST)
    {
        last_ping_ = node_->now();
        has_last_ping_ = true;
        buffer_ = PING_RESPONSE + buffer_;
    }
    else if (data == COMPUTER_KILL_REQUEST)
    {
        set_kill("COMPUTER", true);
        buffer_ = COMPUTER_KILL_RESPONSE + buffer_;
    }
    else if (data == COMPUTER_CLEAR_REQUEST)
    {
        set_kill("COMPUTER", false);
        buffer_ = COMPUTER_CLEAR_RESPONSE + buffer_;
    }
    else if (data == LIGHTS_OFF_REQUEST)
    {
        set_light("OFF");
        buffer_ = LIGHTS_OFF_RESPONSE + buffer_;
    }
    else if (data == LIGHTS_YELLOW_REQUEST)
    {
        set_light("YELLOW");
        buffer_ = LIGHTS_YELLOW_RESPONSE + buffer_;
    }
    else if (data == LIGHTS_GREEN_REQUEST)
    {
        set_light("GREEN");
        buffer_ = LIGHTS_GREEN_RESPONSE + buffer_;
    }
    else
    {
        get_status(data);
    }
}

int SimulatedKillBoard::write(std::string const& data)
{
    check_timeout();

    // Process each byte in the data
    for (char byte : data)
    {
        handle_sync(byte);
    }

    return static_cast<int>(data.length());
}

}  // namespace navigator_kill_board
