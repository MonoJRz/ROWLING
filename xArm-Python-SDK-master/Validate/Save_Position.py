import os
import sys
import time
import math
import socket
import struct
import numpy as np
import transforms3d.euler

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from xarm.wrapper import XArmAPI

# Define the IP address of the robot
ip = "192.168.1.231"

# Connect to the robot
arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.clean_error()
arm.set_mode(0)
arm.set_state(0)
time.sleep(0.1)

########################################################
def bytes_to_fp32(bytes_data, is_big_endian=False):
    return struct.unpack('>f' if is_big_endian else '<f', bytes_data)[0]

def bytes_to_fp32_list(bytes_data, n=0, is_big_endian=False):
    ret = []
    count = n if n > 0 else len(bytes_data) // 4
    for i in range(count):
        ret.append(bytes_to_fp32(bytes_data[i * 4: i * 4 + 4], is_big_endian))
    return ret

def bytes_to_u32(data):
    data_u32 = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]
    return data_u32

robot_ip = ip
robot_port = 30002

# Create socket connection to the control box
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setblocking(True)
sock.settimeout(1)
sock.connect((robot_ip, robot_port))

buffer = sock.recv(4)
while len(buffer) < 4:
    buffer += sock.recv(4 - len(buffer))
size = bytes_to_u32(buffer[:4])
last_digitals = [-1, -1]
buffer += sock.recv(size - len(buffer))         
data = buffer[:size]
buffer = buffer[size:]

# Extract updated position data
buffer += sock.recv(size - len(buffer))
data = buffer[:size]
buffer = buffer[size:]

# Decode the position data
position_data = bytes_to_fp32_list(data[35:59])
x, y, z, roll_rad, pitch_rad, yaw_rad = position_data

# Create a homogeneous transformation matrix from the position data
T_original = np.eye(4)
T_original[0:3, 3] = [x, y, z]

# Convert the Euler angles (roll, pitch, yaw) from radians to a rotation matrix
R = transforms3d.euler.euler2mat(roll_rad, pitch_rad, yaw_rad, 'sxyz')
T_original[0:3, 0:3] = R

# Calculate the extension transformation matrix
d_extension = np.array([0, 0, 150 - 97])  # Update the end-effector length
T_extension = np.eye(4)
T_extension[0:3, 3] = d_extension

# Apply the extension to the original transformation
T_new = np.dot(T_original, T_extension)

# Define the rotation matrix to swap the X and Z axes
Ry_swap = np.array([[0, 0, -1],
                    [0, 1, 0],
                    [1, 0, 0]])
T_swap = np.eye(4)
T_swap[0:3, 0:3] = Ry_swap

# Apply the axis swap transformation
T_new_swapped = np.dot(T_new, T_swap)

# Extract the new position and orientation
new_position = T_new_swapped[0:3, 3]
new_orientation_matrix = T_new_swapped[0:3, 0:3]

# Convert the new orientation matrix back to Euler angles in degrees
new_roll, new_pitch, new_yaw = transforms3d.euler.mat2euler(new_orientation_matrix, 'sxyz')
new_roll_deg = math.degrees(new_roll)
new_pitch_deg = math.degrees(new_pitch)
new_yaw_deg = math.degrees(new_yaw)

# Print the new position and orientation in degrees
print("New position:", new_position)
print("New orientation:", (new_roll_deg, new_pitch_deg, new_yaw_deg))

# Save position data to a text file
with open("position_data.txt", "w") as file:
    file.write(f"x: {x}, y: {y}, z: {z}, roll: {roll_rad}, pitch: {pitch_rad}, yaw: {yaw_rad}\n")


