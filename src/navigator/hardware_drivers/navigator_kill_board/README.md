# Navigator Kill Board

C++ implementation of Navigator's kill board driver and simulation components.

## Overview

This package provides C++ equivalents of the Python kill board simulation and heartbeat functionality for the Navigator autonomous underwater vehicle. The implementation follows the MIL C++ style guide and provides full compatibility with the original Python protocol.

## Components

### Constants (`constants.h`)

Defines all protocol constants used for kill board communication:
- Timeout settings
- Request/response codes
- Kill source definitions
- Light control commands
- Controller protocol definitions

### NoopSerial Class

Base class that provides a no-operation serial interface for testing purposes. All methods are implemented as no-ops, allowing subclasses to override specific functionality.

### SimulatedSerial Class

Extends NoopSerial to provide buffered serial simulation. Maintains an internal buffer that can be written to and read from, simulating real serial device behavior.

### SimulatedKillBoard Class

Full implementation of Navigator's kill board simulation. Features:
- ROS service integration for button simulation
- Timeout-based heartbeat monitoring
- Complete protocol handling (ping, kill, clear, lights)
- State management for all kill sources
- Asynchronous and synchronous communication modes

### HeartbeatServer Class

Publishes periodic heartbeat messages to maintain system connectivity monitoring.

## Usage

### Basic Kill Board Simulation

```cpp
#include "navigator_kill_board/simulated_kill_board.h"

// Create simulated kill board
navigator_kill_board::SimulatedKillBoard kill_board;

// Write protocol data
std::string ping_cmd;
ping_cmd += navigator_kill_board::PING_REQUEST;
kill_board.write(ping_cmd);

// Read response
if (kill_board.in_waiting() > 0) {
  std::string response = kill_board.read(1);
}
```

### Heartbeat Server

```cpp
#include "navigator_kill_board/heartbeat_server.h"

// Create heartbeat server (topic, period_seconds)
navigator_kill_board::HeartbeatServer server("/network", 1.0);
```

## ROS Integration

The SimulatedKillBoard automatically creates ROS services for button simulation:
- `~/BUTTON_FRONT_PORT` (std_srvs/SetBool)
- `~/BUTTON_AFT_PORT` (std_srvs/SetBool)
- `~/BUTTON_FRONT_STARBOARD` (std_srvs/SetBool)
- `~/BUTTON_AFT_STARBOARD` (std_srvs/SetBool)
- `~/BUTTON_REMOTE` (std_srvs/SetBool)

## Example Nodes

### Heartbeat Server Node
```bash
rosrun navigator_kill_board heartbeat_server_node
```

### Simulated Kill Board Node
```bash
rosrun navigator_kill_board simulated_kill_board_node
```

## Testing

Run unit tests:
```bash
catkin_make run_tests_navigator_kill_board
```

## Protocol Documentation

The kill board uses a hex-based serial protocol documented at:
http://docs.mil.ufl.edu/pages/viewpage.action?spaceKey=NAV&title=Kill+Communication+Commands

### Communication Modes

**Ping/Async Mode:**
1. Send PING request (0x20)
2. Continue reading buffer for status responses
3. Parse multiple response bytes for different kill sources

**Sync/Request Mode:**
1. Send specific REQUEST byte for desired kill source
2. Read single byte response (0x00=false, 0x01=true)

### Kill Sources

- OVERALL: Aggregate status of all kill sources
- BUTTON_*: Physical kill buttons on the vehicle
- HEARTBEAT_*: Timeout-based kill sources
- COMPUTER: Software-controllable kill state

## Migration from Python

This C++ implementation maintains full compatibility with the original Python version while providing:
- Better performance and lower latency
- Stronger type safety
- Integration with existing C++ Navigator components
- Memory-efficient operation

## Build Dependencies

- ROS (Melodic/Noetic)
- roscpp
- std_msgs
- std_srvs
- C++11 compiler
