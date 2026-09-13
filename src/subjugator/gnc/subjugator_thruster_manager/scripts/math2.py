import math

import numpy as np
import transforms3d.taitbryan

"""
<xacro:mil_thruster name="FLV" parent="base_link" xyz="0.5715 0.14922 -0.028575" rpy="0 -1.5708 0"/>
<xacro:mil_thruster name="FRV" parent="base_link" xyz="0.5715 -0.14922 -0.028575" rpy="0 -1.5708 0"/>
<xacro:mil_thruster name="BRV" parent="base_link" xyz="0.1143 -0.14922 -0.028575" rpy="0 -1.5708 0"/>
<xacro:mil_thruster name="BLV" parent="base_link" xyz="0.1143 0.14922 -0.028575" rpy="0 -1.5708 0"/>

<xacro:mil_thruster name="FLH" parent="base_link" xyz="0.58737 0.24765 0.053975" rpy="0 0 -0.523598776"/>
<xacro:mil_thruster name="FRH" parent="base_link" xyz="0.58737 -0.24765 0.053975" rpy="0 0 0.523598776"/>
<xacro:mil_thruster name="BRH" parent="base_link" xyz="0.127 -0.24765 0.053975" rpy="0 0 -0.523598776"/>
<xacro:mil_thruster name="BLH" parent="base_link" xyz="0.127 0.24765 0.053975" rpy="0 0 0.523598776"/>
"""

# 30 deg -> 0.5235987755982988 rad

thrusters = np.array(
    [
        [0.58737, -0.24765, 0.053975, 0, 0, 30 * math.pi / 180],  # FRH
        [0.58737, 0.24765, 0.053975, 0, 0, -30 * math.pi / 180],  # FLH
        [0.127, -0.24765, 0.053975, 0, 0, -30 * math.pi / 180],  # BRH
        [0.127, 0.24765, 0.053975, 0, 0, 30 * math.pi / 180],  # BLH
        [0.5715, -0.14922, -0.028575, 0, -math.pi / 2, 0],  # FRV
        [0.5715, 0.14922, -0.028575, 0, -math.pi / 2, 0],  # FLV
        [0.1143, -0.14922, -0.028575, 0, -math.pi / 2, 0],  # BRV
        [0.1143, 0.14922, -0.028575, 0, -math.pi / 2, 0],  # BLV
    ],
)

# origin = [0.357185, 0, 0] # old from math.py
origin = [0.38, 0.005, -0.028575]

np.set_printoptions(suppress=True)

tam = []

for thruster in thrusters:
    r, p, y = thruster[3:]
    thrust = transforms3d.taitbryan.euler2mat(y, p, r) @ [1, 0, 0]

    offset = thruster[:3] - origin

    tam.append(np.concatenate([thrust, np.cross(offset, thrust)]))

tam = np.array(tam).T

for i, name in enumerate(["x", "y", "z", "rx", "ry", "rz"]):
    print(f"      {name}:", [round(x, 10) for x in tam[i]])
