#SAVING LASTEST POSITION AS TXT#
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

angles = arm.get_servo_angle(is_radian=False)
angles_to_save = angles[1][:-1]  # Exclude the last element

# Format the angles as a comma-separated string
angles_str = ", ".join(map(str, angles_to_save))

# Get the directory of the script
script_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(script_dir, "robot_angles.txt")

# Save the angles to a text file
with open(file_path, "w") as file:
    file.write(angles_str)

print("Angles saved to:", file_path)
