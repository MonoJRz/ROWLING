import os
import sys
import time
import math
import socket
import struct
import numpy as np
import time
import transforms3d


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

#########################
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

# Extract updated position data
buffer += sock.recv(size - len(buffer))
data = buffer[:size]
buffer = buffer[size:]

# Decode the position data
position_data = bytes_to_fp32_list(data[35:59])
x, y, z, roll_rad, pitch_rad, yaw_rad = position_data

roll_deg = math.degrees(roll_rad)
pitch_deg = math.degrees(pitch_rad)
yaw_deg = math.degrees(yaw_rad)

ref_frame = 0   


def calculate_vector_and_orientation(P_initial, P_final):
    # Calculate the vector
    V = np.array(P_final) - np.array(P_initial)
    
    # Assuming the vector is initially pointing down the negative Z-axis
    # Calculate angles needed to align the new vector with the calculated one
    theta_y = np.arctan2(-V[0], -V[2])  # Rotation around Y to align with X-Z plane
    theta_x = np.arctan2(V[1], np.sqrt(V[0]**2 + V[2]**2))  # Rotation around X
    
    # Convert radians to degrees
    theta_y = np.degrees(theta_y)
    theta_x = np.degrees(theta_x)
    
    # Enforcing the -90 to 90 range explicitly, if needed
    # Though, due to the nature of atan2, theta_y should already be within this range
    theta_y = np.clip(theta_y, -90, 90)
    
    return V, theta_x, theta_y


# Example usage
# Define other necessary functions and variables (I'll assume you have already defined them)
Adjust1=0
Adjust2=0
# Define the initial and final points
initial_position = np.array([-162.00-Adjust2, 2-Adjust1 ,67.74])
final_position = np.array([-162.00-Adjust2, 2-Adjust1 , 60.74])
(x1,y1,z1) = initial_position
(x2,y2,z2) = final_position
# Example usage
P_initial = (x2, y2, z1)  # Starting at the origin
P_final = (x1, y1, z2)  # Final position in 3D space


vector, rotation_x, rotation_y = calculate_vector_and_orientation(P_initial, P_final)
roll = -rotation_x - 90
yaw  = rotation_y

print("Vector:", vector)
print("Rotation around X-axis:", roll, "degrees")
print("Rotation around Y-axis:", -yaw, "degrees")


# Set the home position
arm.set_position(x, y, z, roll, -yaw, -90, speed=100, wait=True)
arm.set_state(0)
