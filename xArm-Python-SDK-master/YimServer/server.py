
import six
from sksurgerynditracker.nditracker import NDITracker
import keyboard
import time
import numpy as np
import json
from scipy.spatial.transform import Rotation as R
import math
import os 
import socket
import threading

###############NDI setting########################################
SETTINGS1 = {
    "tracker type": "polaris",
    "romfiles": ["C:/Users/FFrist/Desktop/Final/material/8700340.rom","C:/Users/FFrist/Desktop/Final/material/8700339.rom","C:/Users/FFrist/Desktop/Final/material/8700338.rom"],
}   
TRACKER = NDITracker(SETTINGS1)

# Specify the path to the file
directory = 'C:/Users/FFrist/Desktop/Final/material'
file_path = os.path.join(directory, 'C:/Users/FFrist/Desktop/Final/material/transformation_after_ICP.json')
# Assuming `file_path` is defined and points to your JSON file
with open(file_path, 'r') as file:
    array_list = json.load(file)  # Directly load the file into a Python object

# Specify the dtype explicitly if necessary
transform_ICP_spine = np.array(array_list, dtype=float)



###############Data processing###################################################

def robot_relative(robot_h,ref_point):
    Homo_robot=np.array([[1, 0, 0, -100.1],
              [0, 1, 0, 6.2],
              [0, 0, 1, 183.7],
              [0, 0, 0, 1]])
    
    
    Data_point_robot_tip_h = np.dot(robot_h, Homo_robot)

    A_inv = np.linalg.inv(ref_point)
    relative_robot = np.dot(A_inv, Data_point_robot_tip_h)
    
    
    relative_robot_round = np.round(relative_robot, 6)
    print(relative_robot_round)
    return relative_robot_round

def tool_tip_relative(tool_h,ref_point):   
    Homo_tip=np.array([[1, 0, 0, -19.3],
              [0, 1, 0, 1.1],
              [0, 0, 1, -158.1],
              [0, 0, 0, 1]])
    
    
    Data_point_tool_tip_h = np.dot(tool_h, Homo_tip)

    A_inv = np.linalg.inv(ref_point)
    relative_tool = np.dot(A_inv, Data_point_tool_tip_h)
     # Adjusting the position data by dividing by 1000
    relative_tool[:3, 3] /= 1000  # Divide the translation components by 1000``
    relative_tool_round = np.round(relative_tool, 2)
    return relative_tool_round

def spine_pos(transformation_after_ICP_in):
    ref=np.array([[1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, -3],
              [0, 0, 0, 1]])
    
    inspine=ref@transformation_after_ICP_in
     # Adjusting the position data by dividing by 1000
    inspine[:3, 3] /= 1000  # Divide the translation components by 1000
    # Rounding to 5 decimal places
    final_spine_pos_round = np.round(inspine, 6)
    return final_spine_pos_round



def matrix_to_quaternion_translation(h_matrix):
    # Extract rotation matrix
    R = h_matrix[:3, :3]
    # Pre-compute diagonal elements
    t = np.trace(R)
    if t > 0.0:
        S = math.sqrt(t + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    # Extract translation vector
    tx = h_matrix[0, 3]
    ty = h_matrix[1, 3]
    tz = h_matrix[2, 3]

    # Combine into a 7-value representation
    return np.array([qw, qx, qy, qz, tx, ty, tz])

def send_quaternion_data_to_unity_multi(data_array, client_socket):

            
            data_to_send = ','.join(map(str, data_array))

            message = data_to_send + "\n"
            client_socket.send(message.encode('utf-8'))
            print("Data sent to client:", data_to_send)
            time.sleep(0.1)  # Introduce a delay (1 second in this case) to control data rate


##################socket server############################################



# Server configuration
host = '0.0.0.0'  # Listen on all network interfaces
port = 12345

def handle_client(client_socket):
    try:
        
        while True:
            frame_data = TRACKER.get_frame()
            
            Data_point_spine_h=spine_pos(transform_ICP_spine)
            Data_point_spine_q=matrix_to_quaternion_translation(Data_point_spine_h)
            
            Data_point_spine_h_no=(frame_data[3][1])
            Data_point_tool_h_no=(frame_data[3][0])
            Data_point_robot_h_no=(frame_data[3][2])

            Data_point_robot_h=robot_relative(Data_point_robot_h_no,Data_point_spine_h_no)
            Data_point_robot_q=matrix_to_quaternion_translation(Data_point_robot_h)

            Data_point_tool_h=tool_tip_relative(Data_point_tool_h_no,Data_point_spine_h_no)
            Data_point_tool_q=matrix_to_quaternion_translation(Data_point_tool_h)
            
            


            #send data to unity
            data_spine_sent=Data_point_spine_q
            data_tool_sent=Data_point_tool_q
            data_robot_sent=Data_point_robot_q
            
            # Example: Generate or fetch data from NumPy array dynamically
            data_array = np.concatenate((data_spine_sent, data_tool_sent,data_robot_sent))
            
            # Replace NaN values with 0
            data_array_without_nan = np.nan_to_num(data_array, nan=0.0)
            print(data_array)
            

            send_quaternion_data_to_unity_multi(data_array_without_nan,client_socket)
            

    except Exception as e:
        print(f"Client disconnected or error occurred: {e}")
    finally:
        client_socket.close()  # Ensure the socket is closed when the loop exits

def server_loop():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((host, port))
    server.listen(10)  # Max number of connections
    print(f"[*] Listening on {host}:{port}")
    TRACKER.start_tracking()
    print("Connection_data")
    six.print_(TRACKER.get_tool_descriptions())
    print("Position_data")
    while True:
        
        client, addr = server.accept()
        print(f"[*] Accepted connection from: {addr[0]}:{addr[1]}")
        client_handler = threading.Thread(target=handle_client, args=(client,))
        client_handler.start()

if __name__ == "__main__":
    server_loop()
