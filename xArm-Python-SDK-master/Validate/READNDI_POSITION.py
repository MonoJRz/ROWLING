import os
import sys
import time
import math
import socket
import struct
import numpy as np
import threading
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

# Generate Cylinder Boundary:
radius =  0.1 # 10mm in meters
def is_inside_cylinder(point, initial_point, desired_vector, radius):
    projection = initial_point + np.dot(point - initial_point, desired_vector) / np.dot(desired_vector, desired_vector) * desired_vector
    distance = np.linalg.norm(projection - point)
    return distance <= radius

def calculate_deviation_vector(point, initial_point, desired_vector):
    closest_point = closest_point_on_line(point, initial_point, initial_point + desired_vector)
    deviation_vector = closest_point - point
    return deviation_vector

def closest_point_on_line(point, line_point1, line_point2):
    line_vector = line_point2 - line_point1
    projection = line_point1 + np.dot(point - line_point1, line_vector) / np.dot(line_vector, line_vector) * line_vector
    return projection
# Function to read the present position and assign it to NDI_position
def receive_position_data_from_server(server_address, server_port):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect the socket to the server
    server = (server_address, server_port)
    print(f"Connecting to {server_address} port {server_port}")
    sock.connect(server)

    
    # Buffer to hold incoming data
    data_buffer = ""

    # Receiving data from the server
    while True:
        # Receive data from the server
        
        data = sock.recv(1024).decode('utf-8')
        
        data_buffer += data

        # Check if the buffer has at least one complete message
        # Assuming messages are separated by newline characters
        while '\n' in data_buffer:
            # Split on the first newline; everything before it is a complete message
            message, data_buffer = data_buffer.split('\n', 1)

            # Convert the message to a list of floats
            data_list = [float(num) for num in message.split(',') if num.strip()]

            # Check if the list has the expected 21 elements
            if len(data_list) == 21:
                # Extract x, y, z positions from indices 15 to 17
                x, y, z = data_list[18:21]
                # Assign the positions to NDI_position
                NDI_position = (x, y, z)
                print("Received position data:", NDI_position)
                print("Closing connection")
                sock.close()
            else:
                print("Incomplete data received.")

        return NDI_position
    
# Read the present position once and assign it to NDI_position
HOST, PORT = "127.0.0.1", 12345  # Change server_address_here to your server's address
NDI_position =receive_position_data_from_server(HOST, PORT)
print(f"Initial NDI_position: {NDI_position}")