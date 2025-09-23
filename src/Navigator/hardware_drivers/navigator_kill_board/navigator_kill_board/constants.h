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

// Timeout
#define TIMEOUT_SECONDS 8.0

// Responses
#define RESPONSE_FALSE 0x00
#define RESPONSE_TRUE 0x01

// Ping
#define PING_REQUEST 0x20
#define PING_RESPONSE 0x30

// Overall
#define OVERALL_REQUEST 0x21
#define OVERALL_TRUE 0x10
#define OVERALL_FALSE 0x11

// Buttons
#define BUTTON_FRONT_PORT_REQUEST 0x22
#define BUTTON_FRONT_PORT_TRUE 0x12
#define BUTTON_FRONT_PORT_FALSE 0x13

#define BUTTON_AFT_PORT_REQUEST 0x23
#define BUTTON_AFT_PORT_TRUE 0x14
#define BUTTON_AFT_PORT_FALSE 0x15

#define BUTTON_FRONT_STARBOARD_REQUEST 0x24
#define BUTTON_FRONT_STARBOARD_TRUE 0x16
#define BUTTON_FRONT_STARBOARD_FALSE 0x17

#define BUTTON_AFT_STARBOARD_REQUEST 0x25
#define BUTTON_AFT_STARBOARD_TRUE 0x18
#define BUTTON_AFT_STARBOARD_FALSE 0x19

// Heartbeats
#define HEARTBEAT_COMPUTER_REQUEST 0x26
#define HEARTBEAT_COMPUTER_TRUE 0x1A
#define HEARTBEAT_COMPUTER_FALSE 0x1B

#define BUTTON_REMOTE_REQUEST 0x28
#define BUTTON_REMOTE_TRUE 0x3A
#define BUTTON_REMOTE_FALSE 0x3B

#define HEARTBEAT_REMOTE_REQUEST 0x29
#define HEARTBEAT_REMOTE_TRUE 0x3C
#define HEARTBEAT_REMOTE_FALSE 0x3D

// Computer kill/clear
#define COMPUTER_REQUEST 0x27
#define COMPUTER_TRUE 0x1C
#define COMPUTER_FALSE 0x1D

#define COMPUTER_KILL_REQUEST 0x45
#define COMPUTER_KILL_RESPONSE 0x55
#define COMPUTER_CLEAR_REQUEST 0x46
#define COMPUTER_CLEAR_RESPONSE 0x56

// Connection
#define CONNECTED_TRUE 0x1E
#define CONNECTED_FALSE 0x1F

// Lights
#define LIGHTS_OFF_REQUEST 0x40
#define LIGHTS_OFF_RESPONSE 0x50
#define LIGHTS_YELLOW_REQUEST 0x41
#define LIGHTS_YELLOW_RESPONSE 0x51
#define LIGHTS_GREEN_REQUEST 0x42
#define LIGHTS_GREEN_RESPONSE 0x52

// Controller
#define CONTROLLER 0xA0

// Controller sticks (labels only)
#define CTRL_STICK_UD "UD"
#define CTRL_STICK_LR "LR"
#define CTRL_STICK_TQ "TQ"

// Controller buttons (labels only)
#define CTRL_BUTTON_STATION_HOLD "STATION_HOLD"
#define CTRL_BUTTON_RAISE_KILL "RAISE_KILL"
#define CTRL_BUTTON_CLEAR_KILL "CLEAR_KILL"
#define CTRL_BUTTON_THRUSTER_RETRACT "THRUSTER_RETRACT"
#define CTRL_BUTTON_THRUSTER_DEPLOY "THRUSTER_DEPLOY"
#define CTRL_BUTTON_GO_INACTIVE "GO_INACTIVE"
#define CTRL_BUTTON_START "START"
#define CTRL_BUTTON_EMERGENCY_CONTROL "EMERGENCY_CONTROL"

// Controller button values
#define CTRL_BUTTON_STATION_HOLD_VAL 0x0001
#define CTRL_BUTTON_RAISE_KILL_VAL 0x0002
#define CTRL_BUTTON_CLEAR_KILL_VAL 0x0004
#define CTRL_BUTTON_THRUSTER_RETRACT_VAL 0x0010
#define CTRL_BUTTON_THRUSTER_DEPLOY_VAL 0x0020
#define CTRL_BUTTON_GO_INACTIVE_VAL 0x0040
#define CTRL_BUTTON_START_VAL 0x0080
#define CTRL_BUTTON_EMERGENCY_CONTROL_VAL 0x2000
