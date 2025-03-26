# Keyboard Control Package

It's like teleop_twist_keyboard, but better!

- Build with ```colcon build --packages-select subjugator_keyboard_control```
- Run with ```ros2 run subjugator_keyboard_control subjugator_keyboard_control```
- Uses w, a, s, d, z, x for linear movement
- Uses arrows, e, r for angular movement
- Supports key holds
- q for quitting

| Key         | Action                                  |
|-------------|-----------------------------------------|
| **W**       | Move forward  (apply positive X force)  |
| **S**       | Move backward (apply negative X force)  |
| **A**       | Move left     (apply positive Y force)  |
| **D**       | Move right    (apply negative Y force)  |
| **X**       | Move up       (apply positive Z force)  |
| **Z**       | Move down     (apply negative Z force)  |
| ↑ (Up)      | Pitch up      (apply negative Y torque) |
| ↓ (Down)    | Pitch down    (apply positive Y torque) |
| → (Right)   | Yaw right     (apply negative Z torque) |
| ← (Left)    | Yaw left      (apply positive Z torque) |
| **E**       | Roll left     (apply negative X torque) |
| **R**       | Roll left     (apply positive X torque) |
| **Q**       | Quit the loop (`rclcpp::shutdown()`)    |

## Dependencies
- **C++17 or later**
- **ROS 2 (`rclcpp`)** (for `shutdown()` integration)
