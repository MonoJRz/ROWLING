#IMPEDANCEC CONTROL WITH SAFETY BOUNDARY AND ACTIVE CONSTRAIN#
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

# Define the home position angle
#home_position_angle = [3.0, -4.0, 3, 2.3, -90.3, 98.5]


# Define the IP address of the robot
ip = "192.168.1.231"

# Connect to the robot
arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.clean_error()
arm.set_mode(0)
arm.set_state(0)
time.sleep(0.1)

# Set the home position
#arm.set_servo_angle(angle=home_position_angle, speed=20, wait=True)
#arm.set_state(0)
# set pid parameters for force control
Kp = 0.006  # range: 0 ~ 0.05
Ki = 0.00005 # range: 0 ~ 0.0005
Kd = 0.005  # range: 0 ~ 0.05
v_max = 20.0 # max adjust velocity(mm/s), range: 0 ~ 200
arm.set_force_control_pid([Kp]*6, [Ki]*6, [Kd]*6, [v_max]*6)

# enable ft sensor communication
arm.ft_sensor_enable(1)
# will overwrite previous sensor zero and payload configuration
arm.ft_sensor_set_zero()  # remove this if zero_offset and payload already identified & compensated!
time.sleep(0.2)  # wait for writing zero operation to take effect, do not remove

# move robot in impedance control application
arm.ft_sensor_app_set(1)
# will start after set_state(0)
arm.set_state(0)
Input_radius = 50
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

radius = Input_radius   # 10mm in meters

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
# Define other necessary functions and variables (I'll assume you have already defined them)
# Define other necessary functions and variables (I'll assume you have already defined them)
Adjust1=0
Adjust2=0
# Define the initial and final points
initial_position = np.array([-162.00-Adjust2, 2-Adjust1 ,67.74])
final_position = np.array([-162.00-Adjust2, 2-Adjust1 , 60.74])

x1,y1,z1 = initial_position
x2,y2,z2 = final_position
# Define the initial and final points
initial_point = np.array([x1,y1,z1])
final_point = np.array([x2,y2,z2])
previous_error = 0.0  # Initialize previous error
# Calculate the desired vector
desired_vector = final_point - initial_point
#########################
# Initialize variables for tracking previous position and time
previous_position1 = np.array([0, 0, 0])
previous_time1 = time.time()
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

ref_frame = 0   
# Boundary limits
x_max, x_min, y_max, y_min, z_max, z_min = [600, 100, 700, -700, 800, 100]
try:
    current_position = np.array([0, 0, 0])  # Initialize current_position
    while True:
        # Check if the motion mode is activated
        while arm.connected and arm.error_code != 19 and arm.error_code != 28:
            code, digitals = arm.get_tgpio_digital()
            if code == 0:
                if digitals[0] == 1 and digitals[0] != last_digitals[0]:
                    Pause_mode_activated = False
                    print('Motion Mode Activated') 
                    arm.clean_error()
                    arm.set_mode(0)
                    arm.set_state(0)
                    motion_mode_activated = True

                if digitals[0] == 0 and digitals[0] != last_digitals[0]:
                    # Pause the motion
                    motion_mode_activated = False
                    print('Motion Pause')
                    arm.set_mode(1) 
                    Pause_mode_activated = True

                if Pause_mode_activated:
                    # Read the present position once and assign it to NDI_position
                    HOST, PORT = "127.0.0.1", 12345  # Change server_address_here to your server's address
                    NDI_position =receive_position_data_from_server(HOST, PORT)
                    print(f"NDI_position: {NDI_position}")
                    X,Y,Z = NDI_position 

                    # Print the new position and orientation in degrees
                    print("pause")
                if motion_mode_activated:
                    # Print the new position and orientation in degrees
                    print('Running:')
                    # Read the present position once and assign it to NDI_position
                    HOST, PORT = "127.0.0.1", 12345  # Change server_address_here to your server's address
                    NDI_position =receive_position_data_from_server(HOST, PORT)
                    print(f"NDI_position: {NDI_position}")
                    X,Y,Z = NDI_position  
                    # Modify the code to check if the position is inside or outside the cylinder
                    is_inside = is_inside_cylinder(NDI_position, initial_point, desired_vector, radius)
                    # Boundary checking
                        
                    if is_inside:
                            arm.clean_warn()
                            arm.clean_error()
                            arm.motion_enable(enable=True)
                            # remember to reset ft_sensor_app when finished
                            arm.ft_sensor_app_set(0)
                            arm.ft_sensor_enable(0)

                            # Stop generating force by setting the force reference to zero
                            force_ref_zero = [0, 0, 0, 0, 0, 0]
                            force_axis = [1, 1, 1, 0, 0, 0]
                            arm.config_force_control(ref_frame, force_axis, force_ref_zero, [0]*6)

                            # Switch back to impedance control mode
                            # set tool impedance parameters:
                            K_pos = 0         #  x/y/z linear stiffness coefficient, range: 0 ~ 2000 (N/m)
                            K_ori = 0           #  Rx/Ry/Rz rotational stiffness coefficient, range: 0 ~ 20 (Nm/rad)

                            M = float(0.07)  #  x/y equivalent mass; range: 0.02 ~ 1 kg
                            Mz = float(0.11)  #  z equivalent mass; range: 0.02 ~ 1 kg
                            J = M * 0.05     #  Rx/Ry/Rz equivalent moment of inertia, range: 1e-4 ~ 0.01 (Kg*m^2)

                            c_axis = [1, 1, 1, 1, 1, 1]  # set z axis as compliant axis
                            ref_frame = 0         # 0 : base , 1 : tool
                            arm.set_state(0)
                            arm.set_mode(0)
                            arm.set_impedance_mbk([M, M, Mz, J, J, J], [K_pos, K_pos, K_pos, K_ori, K_ori, K_ori], [0]*6)  # Reset impedance parameters
                            arm.set_impedance_config(ref_frame, c_axis)  # Reset impedance configuration
                            # Stop generating force by setting the force reference to zero
                            force_ref_zero = [0, 0, 0, 0, 0, 0]
                            force_axis = [1, 1, 1, 1, 1, 1]
                            arm.config_force_control(ref_frame, force_axis, force_ref_zero, [0]*6)
                            # Enable the force sensor application for impedance control
                            arm.ft_sensor_enable(1)
                            # wait for writing zero operation to take effect, do not remove
                            # Resume impedance control application
                            arm.ft_sensor_app_set(1)
                            arm.set_state(0)
                            
                    if not is_inside:
                            arm.clean_warn()
                            arm.clean_error()
                            arm.motion_enable(enable=True)
                            # Read the present position once and assign it to NDI_position
                            HOST, PORT = "127.0.0.1", 12345  # Change server_address_here to your server's address
                            NDI_position =receive_position_data_from_server(HOST, PORT)
                            print(f"Out_Of_PATH_NDI_position: {NDI_position}")
                            X,Y,Z = NDI_position
                            

                            # Calculate deviation vector
                            deviation_vector = calculate_deviation_vector( NDI_position, initial_point, desired_vector)

                            # Calculate error between current position and desired path
                            error = np.linalg.norm(deviation_vector) 

                            # Calculate corrective force direction
                            force_direction = deviation_vector / np.linalg.norm(deviation_vector)

                            # Calculate corrective force magnitude using a PD controller
                            Kp = 0.35
                            Kd = 0.0
                            dt = 0.01  # Time step
                            derivative_term = Kd * (error - previous_error) / dt
                            corrective_force_magnitude = Kp * error + derivative_term

                            # Apply corrective force to robot
                            corrective_force = corrective_force_magnitude * force_direction

                            # Calculate corrective force components along x, y, and z axes
                            force_y = corrective_force[0]
                            force_x = corrective_force[1]
                            force_z = corrective_force[2]

                            # Clamp the values of the corrective force components
                            max_force = 20
                            min_force = -20

                            # Clamp the force components to the specified range
                            force_x = max(min(force_x, max_force), min_force)
                            force_y = max(min(force_y, max_force), min_force)
                            force_z = max(min(force_z, max_force), min_force)

                            force_ref = [force_x, force_y, force_z, 0, 0, 0]
                            force_axis = [1, 1, 1, 0, 0, 0]
                            arm.config_force_control(ref_frame,  force_axis, force_ref, [0]*6)
                            # enable ft sensor communication
                            arm.ft_sensor_enable(1)
                            arm.ft_sensor_app_set(2)
                            # will start after set_state(0)
                            arm.set_state(0)

                            # Print the components
                            print("Corrective Force Components:")
                            print("Force along X-axis:", force_x)
                            print("Force along Y-axis:", force_y)
                            print("Force along Z-axis:", force_z)

                            # Update previous error for next iteration
                            previous_error = error
                                
            last_digitals = digitals
            time.sleep(0.03)  # Add a small delay to avoid high CPU usage

except KeyboardInterrupt:
    pass  # Catch keyboard interrupt to gracefully exit the loop

# Remember to reset ft_sensor_app when finished
arm.ft_sensor_app_set(0)
arm.ft_sensor_enable(0)
arm.disconnect()
