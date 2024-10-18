#READING NDI DATA#
import socket
import six
from sksurgerynditracker.nditracker import NDITracker
import keyboard
import time
import numpy as np
import json
from scipy.spatial.transform import Rotation as R
import math

SETTINGS1 = {
    "tracker type": "polaris",
    "romfiles": ["D:/thesis/thesis/navi/readNDI/8700338.rom","D:/thesis/thesis/navi/readNDI/8700339.rom"],
}   

TRACKER = NDITracker(SETTINGS1)

# Define server addresses including HoloLens 2
servers = [
    ('127.0.0.1', 12345),  # Server 1 address (e.g., your Unity server on the same machine)
    ('192.168.1.32', 12345)  # Server 2 address (HoloLens 2)
]

def tool_tip_relative(tool_h,ref_point):   
    #determine the homogeneous transformation matrix to the tip 
    Homoholder_tip=np.array([[1, 0, 0, -19.3],
              [0, 1, 0, 1.1],
              [0, 0, 1, -158.1],
              [0, 0, 0, 1]])
    Data_point_tool_tip_h = np.dot(tool_h, Homoholder_tip)

    A_inv = np.linalg.inv(ref_point)
    relative_tool = np.dot(A_inv, Data_point_tool_tip_h)
     # Adjusting the position data by dividing by 1000
    relative_tool[:3, 3] /= 1000  # Divide the translation components by 1000``

    relative_tool_round = np.round(relative_tool, 5)
    return relative_tool_round

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

def send_quaternion_data_to_unity_multi(Data_point_tool_q_no, client_sockets):
    tooldata = np.concatenate( Data_point_tool_q_no)
    quaternion_data = ','.join(map(str, tooldata))
    data_to_send = f"{quaternion_data}\n"  # Add a newline for easier parsing on the receiving end

    for socket in client_sockets:
        try:
            socket.send(data_to_send.encode('utf-8'))
        except Exception as e:
            print(f"Error sending data to Unity server: {e}")
        # Control the frame rate (adjust sleep duration accordingly)
        time.sleep(0.5)

# Initialize a list to hold the client sockets for each server connection
client_sockets = []
for server in servers:
    try:
        # Create a new socket for each server
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect to the server
        client_socket.connect(server)
        print(f"Connected to server at {server[0]}:{server[1]}")
        # Store the socket in the list
        client_sockets.append(client_socket)
    except Exception as e:
        print(f"Error connecting to server at {server[0]}:{server[1]}: {e}")


try:
    TRACKER.start_tracking()
    print("Connection_data")
    six.print_(TRACKER.get_tool_descriptions())
    print("Position_data")

    while not keyboard.is_pressed('esc'):
        frame_data = TRACKER.get_frame()
        
        #extract q and t from NDI and sent the data that is not error
       
        
        #pure POSE
        Data_point_spine_h_no=(frame_data[3][1])#reference marker point
        Data_point_tool_h_no=(frame_data[3][0])#Marker on the robot


        Data_point_tool_h=tool_tip_relative(Data_point_tool_h_no,Data_point_spine_h_no)
        Data_point_tool_q=matrix_to_quaternion_translation(Data_point_tool_h)
        
        #to check the avialable of the data is that any "nan" in the data to send
        check_variable_tool=str(Data_point_tool_q[1])
        error="nan"

        #send data to other program
        data_tool_sent=Data_point_tool_q

        if check_variable_tool== error  :
            data_tool_sent=[ 0 ,0, 0, 0,0, 0,0]
            send_quaternion_data_to_unity_multi(data_tool_sent,client_sockets)
            print ("no data_tool")
        
        else :
            send_quaternion_data_to_unity_multi(data_tool_sent,client_sockets)
        
        
        

except Exception as e:
    print(f"Error: {e}")

finally:
    TRACKER.stop_tracking()
    TRACKER.close()
    for client_socket in client_sockets:
        client_socket.close()
