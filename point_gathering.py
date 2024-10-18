import six
from sksurgerynditracker.nditracker import NDITracker
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from random import randint
from pynput import keyboard
import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d
import os
from NDIconnection import Capture
# Initialize a list to store the print statements
print_statements = []
coordinates = []
capture = Capture()
capture.connect()
capture.init_polaris()
print_statements.append("Connection_data")
print_statements.append("Position_data")

Homo_tip=np.array([[1, 0, 0, -19.3],
              [0, 1, 0, 1.1],
              [0, 0, 1, -158.1],
              [0, 0, 0, 1]])





    
# Function to update the 3D plot in real-time
def update_plot(frame):
    if key_pressed:
        result = realtime_point()
        # Check if result is not None
        if result is not None:
            x, y, z = result
            ax.scatter(x, y, z, color='red')
            ax.set_title(f'Real-Time 3D Point: ({x}, {y}, {z})')
            coordinates.append((x, y, z))
            print(len(coordinates))
        else:
            return




def quaternion_to_transformation_matrix(w, x, y, z, tx, ty, tz):
    # Placeholder implementation - replace with actual function
    rotation = R.from_quat([x, y, z, w]).as_matrix()
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation
    transformation_matrix[:3, 3] = [tx, ty, tz]
    return transformation_matrix

def realtime_point():
    capture.readdata()
    frame_data = capture.get_quanternion_data()
    t = frame_data
    
    # Extract q and t from NDI
    Data_point_tool_q = (t[0])
    Data_point_ref_q = (t[1])
    # print(Data_point_tool_q)
    # print(Data_point_ref_q)
    # Check if any quaternion or translation vector component is NaN
    if (Data_point_tool_q[6])==0 or (Data_point_ref_q[6])==0:
        
        return
    else:
        #define for quanternion converter
        w1, x1, y1, z1, t_x1, t_y1, t_z1 = Data_point_tool_q
        w2, x2, y2, z2, t_x2, t_y2, t_z2 = Data_point_ref_q

        #Homogenous of the points
        Data_point_tool_h = quaternion_to_transformation_matrix(w1, x1, y1, z1, t_x1, t_y1, t_z1)
        Data_point_ref_h = quaternion_to_transformation_matrix(w2, x2, y2, z2, t_x2, t_y2, t_z2)

        #combine tool tip to tool
        Data_point_tool_tip_h = np.dot(Data_point_tool_h, Homo_tip)
        
        # Step 1: Compute the inverse of the reference frame matrix
        Data_point_ref_inv_h = np.linalg.inv(Data_point_ref_h)

        Data_point_tool_tip_h_relative = np.dot(Data_point_ref_inv_h, Data_point_tool_tip_h)
        relative_position_x = Data_point_tool_tip_h_relative[0][3]
        relative_position_y = Data_point_tool_tip_h_relative[1][3]
        relative_position_z =Data_point_tool_tip_h_relative[2][3]
       
        return (relative_position_x), (relative_position_y), (relative_position_z)

   
# Function to handle key press events
def on_press(key):
    global key_pressed
    if key == keyboard.Key.space:
        key_pressed = True

# Function to handle key release events
def on_release(key):
    global key_pressed, continue_animation
    if key == keyboard.Key.space:
        key_pressed = False
    elif key == keyboard.Key.esc:
        continue_animation = False
        save_pcd_file("gather_point", coordinates)
        capture.disconnect()
        plt.close('all')

#save data to .xyz file

def save_pcd_file(file_name, coordinates, folder_path="C:/Users/BARTLAB/Desktop/Final (2)/Final/material"):
    # Ensure the folder exists
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    
    # Generate the full file path
    file_path = os.path.join(folder_path, f"{file_name}.pcd")
    
    # Convert coordinates to Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(coordinates)
    
    # Save the point cloud to a PCD file
    o3d.io.write_point_cloud(file_path, pcd)
    print(f"The output is saved as a PCD file at {file_path}.")
           
# Create a figure and a 3D axis
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')


# Flag to control whether the animation should continue
continue_animation = True

# Flag to track space bar press
key_pressed = False
# Start listening for key presses
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# Create an animation that calls the update_plot function every 1000 milliseconds
ani = FuncAnimation(fig, update_plot, interval=0.1)

# Set axis labels
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')

# Display the plot
plt.show()

# Run the animation until the space bar is pressed
while continue_animation:
    plt.pause(1)


listener.stop()
listener.join()
