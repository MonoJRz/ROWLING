#STOP FUNCTION#
import os
import sys
import time
import math
import socket
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from xarm.wrapper import XArmAPI

# Define the IP address of the robot
ip = "192.168.1.231"   

# Connect to the robot
arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.emergency_stop()
arm.disconnect()