#AUTO ENTERING WITH MODIFIED TOOL TIP#
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

# Define the initial and final points
initial_point = np.array([535.71832903,  131.40351008, 225.31497096])
final_point = np.array([535.71832903,  131.40351008, 200.31497096])


# Calculate the desired vector
desired_vector = final_point - initial_point

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



while True:
    buffer += sock.recv(size - len(buffer))
    if len(buffer) < size:
        continue
    data = buffer[:size]
    buffer = buffer[size:]
    # Decode the position and orientation data
    position_data = bytes_to_fp32_list(data[35:59])
    position_data_rounded = [round(val, 1) for val in position_data]
    x, y, z, roll_rad, pitch_rad, yaw_rad = position_data

    # Create a homogeneous transformation matrix from the position and orientation data
    T_original = np.eye(4)
    T_original[0:3, 3] = [x, y, z]

    # Convert the Euler angles (roll, pitch, yaw) from radians to a rotation matrix
    R = transforms3d.euler.euler2mat(roll_rad, pitch_rad, yaw_rad, 'sxyz')
    T_original[0:3, 0:3] = R

    # Calculate the extension transformation matrix
    d_extension = np.array([0, 0, 100.00 + 97])  # Update the end-effector length
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
    point = new_position
    # Print the new position and orientation in degrees
    print('Running:')
    print("New position:", new_position)
    print("New orientation:", (new_roll_deg, new_pitch_deg, new_yaw_deg))
    # Modify the code to check if the position is inside or outside the cylinder
    is_inside = is_inside_cylinder(point, initial_point, desired_vector, radius)
    # Boundary checking
    x1, y1, z1 = new_position
    roll1 = new_roll_deg
    pitch1 = new_pitch_deg
    yaw1 = new_yaw_deg

    

    if is_inside:
        print("Inside Corrective Path", point)
        # Remember to reset ft_sensor_app when finished
        arm.ft_sensor_app_set(0)
        arm.ft_sensor_enable(0)
        arm.disconnect()
        break  # Exit the loop if inside the cylinder

    else:
        print("Outside Corrective Path", point)
        
        