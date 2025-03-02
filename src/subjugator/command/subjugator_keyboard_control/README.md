# Keyboard Control Package

It's like teleop_twist_keyboard, but better!

- Build with ```colcon build --packages-select subjugator_keyboard_control```
- Run with ```ros2 run subjugator_keyboard_control subjugator_keyboard_control```
- Uses arrows (up, down, left, right, shift+up, shift+down) for linear movement.
- Uses wasd (w, a, s, d, shift+w, shift+s) for angular movement
- Supports key holds and pressing space to reset
- q for quitting


| Key         | Action                               |
|-------------|--------------------------------------|
| ↑ (Up)      | Move forward (increase X force)      |
| ↓ (Down)    | Move backward (decrease X force)     |
| → (Right)   | Move right (increase Y force)        |
| ← (Left)    | Move left (decrease Y force)         |
| **Shift+↑** | Move up (increase Z force)           |
| **Shift+↓** | Move down (decrease Z force)         |
| **W**       | Apply torque in X+ direction         |
| **S**       | Apply torque in X- direction         |
| **A**       | Apply torque in Z- direction         |
| **D**       | Apply torque in Z+ direction         |
| **Shift+W** | Apply torque in Y+ direction         |
| **Shift+S** | Apply torque in Y- direction         |
| **Space**   | Stop all motion (reset key states)   |
| **Q**       | Quit the loop (`rclcpp::shutdown()`) |

## Dependencies
- **C++17 or later**
- **ROS 2 (`rclcpp`)** (for `shutdown()` integration)
