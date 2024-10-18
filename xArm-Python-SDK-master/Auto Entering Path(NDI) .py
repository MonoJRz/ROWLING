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
radius =  20 # 10mm in meters
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

def receive_position_data_from_server2(server_address, server_port):
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

        return NDI_position1
      
# Set Cartesian velocity control mode
arm.set_mode(5)
arm.set_state(0)
time.sleep(0.1)
# Maximum velocity
max_velocity = 20
arm.clean_error()
arm.motion_enable(enable=True)
# Read the present position once and assign it to NDI_position
HOST, PORT = "127.0.0.1", 12345  # Change server_address_here to your server's address
NDI_position =receive_position_data_from_server(HOST, PORT)
print(f"Initial NDI_position: {NDI_position}")

########################################################
# Define other necessary functions and variables (I'll assume you have already defined them)
Adjust1=-45
Adjust2=-40
# Define the initial and final points
initial_position = np.array([-162.00-Adjust2, 2-Adjust1 ,67.74])
final_position = np.array([-162.00-Adjust2, 2-Adjust1 , 60.74])

x1,y1,z1 = initial_position
x2,y2,z2 = final_position
# Define the initial and final points
initial_point = np.array([x1,y1,z1])
final_point = np.array([x2,y2,z2])
# Calculate the desired vector
desired_vector = final_point - initial_point

# Define radius for the cylinder
radius = 15  # 10mm in meters

# Define maximum velocity
max_velocity = 20

# Define proportional gain for the PD controller
Kp = 0.7


# Calculate deviation vector
deviation_vector = calculate_deviation_vector(NDI_position, initial_point, desired_vector)
error = np.linalg.norm(deviation_vector)
print("Vector",deviation_vector)
# Calculate velocity direction
velocity_direction = deviation_vector / np.linalg.norm(deviation_vector)

# Calculate desired velocity magnitude using a PD controller (assuming proportional control only)
desired_velocity_magnitude = Kp * error

# Ensure desired velocity magnitude does not exceed max_velocity
if desired_velocity_magnitude > max_velocity:
    desired_velocity_magnitude = max_velocity

# Calculate time to reach the position
time_to_reach = error / (desired_velocity_magnitude + 0.0001)  # added small value to avoid division by zero

print("velocity_directionx",velocity_direction[0])
print("velocity_directiony",velocity_direction[1])
print("velocity_directionz",velocity_direction[2])
# Calculate velocity components along x, y, and z axes
velocity_x = desired_velocity_magnitude * velocity_direction[0]
velocity_y = desired_velocity_magnitude * -velocity_direction[1]
velocity_z = desired_velocity_magnitude * velocity_direction[2]

# Set Cartesian velocity
arm.vc_set_cartesian_velocity([velocity_y, velocity_x, velocity_z, 0, 0, 0])
time.sleep(time_to_reach)

# Stop
arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0])
NDI_position1 =receive_position_data_from_server(HOST, PORT)
# Check if the position is inside the cylinder
is_inside = is_inside_cylinder(NDI_position1, initial_point, desired_vector, radius)
print("Final_position",NDI_position1)

if is_inside:
    # Remember to reset ft_sensor_app when finished
    print("Inside Corrective Path", NDI_position1)
    arm.ft_sensor_app_set(0)
    arm.ft_sensor_enable(0)
    arm.disconnect()

else:
    print("Outside Corrective Path", NDI_position1)
    

