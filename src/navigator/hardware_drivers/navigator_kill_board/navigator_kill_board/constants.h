/*
Navigator's kill board communicates over serial using the hex codes listed below.

This should be the same as the codes on:
    http://docs.mil.ufl.edu/pages/viewpage.action?spaceKey=NAV&title=Kill+Communication+Commands

The board can operate both in a request / response way, or it can be
periodically pinged and will respond with many bytes indicating the status
of all addresses.

For the ping/Async method, send PING, than continue to read the buffer
and parse the various response bytes below.

For the sync / request method, send the REQUEST byte for whatever addresses you
are interested in, then read a 1 byte response for either RESPONSE_FALSE or RESPONSE_TRUE

The computer can also command a kill (for example, if ROS notices a criticaly low battery)
by sending the COMPUTER.KILL.REQUEST and undone with COMPUTER.CLEAR.REQUEST
*/

#pragma once

#include <string>
#include <vector>

namespace navigator_kill_board
{

// Timeout - How often board must be pinged to not set HEARTBEAT_REMOTE True
double const TIMEOUT_SECONDS = 8.0;

// Response codes for synchronous requests
char const RESPONSE_FALSE = 0x00;
char const RESPONSE_TRUE = 0x01;

// Ping protocol
char const PING_REQUEST = 0x20;
char const PING_RESPONSE = 0x30;

// Overall kill status - Should be True if any of the others are True
char const OVERALL_REQUEST = 0x21;
char const OVERALL_TRUE = 0x10;
char const OVERALL_FALSE = 0x11;

// Button kill sources
char const BUTTON_FRONT_PORT_REQUEST = 0x22;
char const BUTTON_FRONT_PORT_TRUE = 0x12;
char const BUTTON_FRONT_PORT_FALSE = 0x13;

char const BUTTON_AFT_PORT_REQUEST = 0x23;
char const BUTTON_AFT_PORT_TRUE = 0x14;
char const BUTTON_AFT_PORT_FALSE = 0x15;

char const BUTTON_FRONT_STARBOARD_REQUEST = 0x24;
char const BUTTON_FRONT_STARBOARD_TRUE = 0x16;
char const BUTTON_FRONT_STARBOARD_FALSE = 0x17;

char const BUTTON_AFT_STARBOARD_REQUEST = 0x25;
char const BUTTON_AFT_STARBOARD_TRUE = 0x18;
char const BUTTON_AFT_STARBOARD_FALSE = 0x19;

// Heartbeat monitoring - Will return True if not pinged often enough
char const HEARTBEAT_COMPUTER_REQUEST = 0x26;
char const HEARTBEAT_COMPUTER_TRUE = 0x1A;
char const HEARTBEAT_COMPUTER_FALSE = 0x1B;

char const BUTTON_REMOTE_REQUEST = 0x28;
char const BUTTON_REMOTE_TRUE = 0x3A;
char const BUTTON_REMOTE_FALSE = 0x3B;

char const HEARTBEAT_REMOTE_REQUEST = 0x29;
char const HEARTBEAT_REMOTE_TRUE = 0x3C;
char const HEARTBEAT_REMOTE_FALSE = 0x3D;

// Computer kill/clear - Allows board to be killed over serial
char const COMPUTER_REQUEST = 0x27;
char const COMPUTER_TRUE = 0x1C;
char const COMPUTER_FALSE = 0x1D;

char const COMPUTER_KILL_REQUEST = 0x45;
char const COMPUTER_KILL_RESPONSE = 0x55;
char const COMPUTER_CLEAR_REQUEST = 0x46;
char const COMPUTER_CLEAR_RESPONSE = 0x56;

// Connection status
char const CONNECTED_TRUE = 0x1E;
char const CONNECTED_FALSE = 0x1F;

// Light control - YELLOW turns off GREEN and vice versa
char const LIGHTS_OFF_REQUEST = 0x40;
char const LIGHTS_OFF_RESPONSE = 0x50;
char const LIGHTS_YELLOW_REQUEST = 0x41;
char const LIGHTS_YELLOW_RESPONSE = 0x51;
char const LIGHTS_GREEN_REQUEST = 0x42;
char const LIGHTS_GREEN_RESPONSE = 0x52;

// Controller - Signifies start of controller message (joysticks & buttons)

// Controller sticks (labels only)
char const* const CTRL_STICK_UD = "UD";
char const* const CTRL_STICK_LR = "LR";
char const* const CTRL_STICK_TQ = "TQ";

// Controller buttons (labels only)
char const* const CTRL_BUTTON_STATION_HOLD = "STATION_HOLD";
char const* const CTRL_BUTTON_RAISE_KILL = "RAISE_KILL";
char const* const CTRL_BUTTON_CLEAR_KILL = "CLEAR_KILL";
char const* const CTRL_BUTTON_THRUSTER_RETRACT = "THRUSTER_RETRACT";
char const* const CTRL_BUTTON_THRUSTER_DEPLOY = "THRUSTER_DEPLOY";
char const* const CTRL_BUTTON_GO_INACTIVE = "GO_INACTIVE";
char const* const CTRL_BUTTON_START = "START";
char const* const CTRL_BUTTON_EMERGENCY_CONTROL = "EMERGENCY_CONTROL";

// Controller - Signifies start of controller message (joysticks & buttons)
unsigned char const CONTROLLER = 0xA0;

// Controller stick names
std::vector<std::string> const CTRL_STICKS = { "UD", "LR", "TQ" };

// Controller button names
std::vector<std::string> const CTRL_BUTTONS = { "STATION_HOLD",    "RAISE_KILL",  "CLEAR_KILL", "THRUSTER_RETRACT",
                                                "THRUSTER_DEPLOY", "GO_INACTIVE", "START",      "EMERGENCY_CONTROL" };

// Controller button values (16-bit values)
unsigned short const CTRL_BUTTON_STATION_HOLD_VAL = 0x0001;
unsigned short const CTRL_BUTTON_RAISE_KILL_VAL = 0x0002;
unsigned short const CTRL_BUTTON_CLEAR_KILL_VAL = 0x0004;
unsigned short const CTRL_BUTTON_THRUSTER_RETRACT_VAL = 0x0010;
unsigned short const CTRL_BUTTON_THRUSTER_DEPLOY_VAL = 0x0020;
unsigned short const CTRL_BUTTON_GO_INACTIVE_VAL = 0x0040;
unsigned short const CTRL_BUTTON_START_VAL = 0x0080;
unsigned short const CTRL_BUTTON_EMERGENCY_CONTROL_VAL = 0x2000;

// Kill source names for iteration
std::vector<std::string> const KILLS = { "OVERALL",
                                         "BUTTON_FRONT_PORT",
                                         "BUTTON_AFT_PORT",
                                         "BUTTON_FRONT_STARBOARD",
                                         "BUTTON_AFT_STARBOARD",
                                         "HEARTBEAT_COMPUTER",
                                         "BUTTON_REMOTE",
                                         "HEARTBEAT_REMOTE",
                                         "COMPUTER" };

}  // namespace navigator_kill_board
