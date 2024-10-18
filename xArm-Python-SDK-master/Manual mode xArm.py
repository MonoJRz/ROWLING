#MANUAL MODE ACTIVATED#
import os
import sys
import time
import math
import socket
import struct
import numpy as np
import time


sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from xarm.wrapper import XArmAPI

# Define the IP address of the robot
ip = "192.168.1.231"

# Connect to the robot
arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.clean_error()
arm.set_state(0)
time.sleep(0.1)

arm.set_mode(2)

#########################
robot_ip = ip
robot_port = 30002

# Create socket connection to the control box
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setblocking(True)
sock.settimeout(1)
sock.connect((robot_ip, robot_port))


