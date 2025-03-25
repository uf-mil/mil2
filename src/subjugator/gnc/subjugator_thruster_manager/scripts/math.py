import math

center = (0.254, 0, 0)

print("Horizontal thrusters (angled)\n")

position = (0.58737, 0.24765, 0.053975)
orientation = (0, 0, 330)  # degrees

dist_xy = math.dist(center[:2], position[:2])
theta = (
    2 * math.pi - math.radians(orientation[2]) + math.atan(position[1] / position[0])
)
torque_z_yaw = dist_xy * math.sin(theta)
print(f"x: {math.cos(math.radians(30))}")
print(f"y: {math.sin(math.radians(30))}")
print(f"torque_z_yaw: {torque_z_yaw}")

position = (0.58737, -0.24765, 0.053975)
orientation = (0, 0, -330)  # degrees

dist_xy = math.dist(center[:2], position[:2])
theta = (
    2 * math.pi - math.radians(orientation[2]) + math.atan(position[1] / position[0])
)
torque_z_yaw = dist_xy * math.sin(theta)
print(f"torque_z_yaw: {torque_z_yaw}")

position = (0.127, 0.24765, 0.053975)
orientation = (0, 0, 30)  # degrees

dist_xy = math.dist(center[:2], position[:2])
theta = (
    math.pi / 2 - math.radians(orientation[2]) + math.atan(position[0] / position[1])
)
torque_z_yaw = dist_xy * math.sin(theta)
print(f"x: {math.cos(math.radians(30))}")
print(f"y: {math.sin(math.radians(30))}")
print(f"torque_z_yaw: {torque_z_yaw}")

position = (0.127, -0.24765, 0.053975)
orientation = (0, 0, -30)  # degrees

dist_xy = math.dist(center[:2], position[:2])
theta = (
    math.pi / 2 - math.radians(orientation[2]) + math.atan(position[0] / position[1])
)
torque_z_yaw = dist_xy * math.sin(theta)
print(f"torque_z_yaw: {torque_z_yaw}")
