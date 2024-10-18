#MAKE THE ROBOT GO BACK TO LASTEST POSITION THAT HAS BEEN SAVED#
import os
import sys
import time
import math
import socket
import struct
import numpy as np
from xarm.wrapper import XArmAPI

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

robot_ip = "192.168.1.231"
robot_port = 30002

# Create socket connection to the control box
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setblocking(True)
sock.settimeout(1)
sock.connect((robot_ip, robot_port))

def read_angles_from_file(file_path):
    try:
        with open(file_path, "r") as file:
            angles_str = file.readline().strip()  # Read the first line
            angles = [float(angle.strip()) for angle in angles_str.split(",")]
            return angles
    except FileNotFoundError:
        print("Error: File not found.")
        return None
    except Exception as e:
        print("Error:", e)
        return None

# Define the IP address of the robot
ip = "192.168.1.231"

# Connect to the robot
arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.clean_error()
arm.set_mode(0)
arm.set_state(0)
time.sleep(0.1)

# Read angles from file
script_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(script_dir, "robot_angles.txt")
saved_position_angle = read_angles_from_file(file_path)

if saved_position_angle is not None:
    # Set the home position
    arm.set_servo_angle(angle=saved_position_angle, speed=10, wait=True)
    arm.set_state(0)

# Remember to reset ft_sensor_app when finished
arm.ft_sensor_app_set(0)
arm.ft_sensor_enable(0)
arm.disconnect()
