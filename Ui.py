from tkinter import messagebox
import customtkinter
import threading
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d
from queue import Queue
import time
from sksurgerynditracker.nditracker import NDITracker
import subprocess
from NDIconnection import Capture
import os

customtkinter.set_appearance_mode("System")  # Or "Dark", "Light"
customtkinter.set_default_color_theme("blue")

class TrackingApplication(customtkinter.CTk):
   
    def __init__(self):
        super().__init__()
        self.title("NDI Optical Camera Tracking")
        self.geometry("1200x600")

        self.capture = None
        self.Homo_tip = np.array([[1, 0, 0, -19.3],
                                  [0, 1, 0, 1.1],
                                  [0, 0, 1, -158.1],
                                  [0, 0, 0, 1]])

        
        #self.tracker = None
        self.setup_ui()
        self.update_queue = Queue()
        self.coordinates = []
        self.is_tracking = False  # Add this line to track the state
        self.is_paused = False  # Initialize the pause state
        self.server_process = None  # To keep track of the server subprocess

        self.protocol("WM_DELETE_WINDOW", self.on_closing)  # Proper shutdown

    def setup_ui(self):
        self.frame_left = customtkinter.CTkFrame(master=self, width=200, corner_radius=10)
        self.frame_left.grid(row=0, column=0, sticky="nswe", padx=20, pady=20)
        self.frame_left.grid_rowconfigure(1, weight=1)
        self.frame_left.grid_columnconfigure(0, weight=1)

        self.button_container = customtkinter.CTkFrame(master=self.frame_left)
        self.button_container.grid(row=0, column=0, sticky="nsew")
        self.button_container.grid_rowconfigure([0, 4], weight=1)  # Add spacing above and below
        self.button_container.grid_columnconfigure(0, weight=1)

        self.button_start = customtkinter.CTkButton(master=self.button_container, text="Start", command=self.start_tracking)
        self.button_start.grid(row=1, column=0, pady=20, padx=20, sticky="ew")
        self.button_pause = customtkinter.CTkButton(master=self.button_container, text="Pause", command=self.toggle_pause)
        self.button_pause.grid(row=2, column=0, pady=20, padx=20, sticky="ew")
        self.button_reset = customtkinter.CTkButton(master=self.button_container, text="Reset", command=self.reset_tracking)
        self.button_reset.grid(row=3, column=0, pady=20, padx=20, sticky="ew")
        self.button_complete = customtkinter.CTkButton(master=self.button_container, text="Complete", command=self.complete_tracking)
        self.button_complete.grid(row=4, column=0, pady=60, padx=20, sticky="ew")

        
        self.appearance_mode_label = customtkinter.CTkLabel(self.frame_left, text="Appearance Mode:", anchor="w")
        self.appearance_mode_label.grid(row=5, column=0, padx=(20, 20), pady=(20, 0))
        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(self.frame_left, values=["Light", "Dark", "System"],
                                                                       command=self.change_appearance_mode)
        self.appearance_mode_optionemenu.grid(row=6, column=0, padx=(20, 20), pady=(10, 20))


        self.frame_right = customtkinter.CTkFrame(master=self, corner_radius=10)
        self.frame_right.grid(row=0, column=1, sticky="nswe", padx=20, pady=20)
        self.grid_columnconfigure(1, weight=5)
        self.grid_rowconfigure(0, weight=1)

        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame_right)
        self.canvas.draw()
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill="both", expand=True)
  
        
    def create_tracking_thread(self):
         # Thread creation for tracking and updating the plot
        self.tracking_thread = threading.Thread(target=self.track_and_update_plot, daemon=True)

    def start_tracking(self):
        # Starts tracking using the Capture class
        if self.capture:
            self.capture.disconnect()
        self.capture = Capture()
        self.capture.connect()
        self.capture.init_polaris()
        self.is_tracking = True# Update tracking state to True when starting
        self.create_tracking_thread()
        self.tracking_thread.start()

    def toggle_pause(self):
    # Toggle the pause state and update the button text
        self.is_paused = not self.is_paused
        if self.is_paused:
            self.button_pause.configure(text="Resume")
        else:
            self.button_pause.configure(text="Pause")
        # Resume the tracking process if it was paused
            if not self.tracking_thread.is_alive():
                self.create_tracking_thread()
                self.tracking_thread.start()


    def track_and_update_plot(self):
    # Continuously track data and update plot unless paused
        while self.is_tracking:
            if not self.is_paused:
                data_point = self.realtime_point()
            if data_point:
                self.update_queue.put(data_point)
            time.sleep(0.05)  # Adjust as necessary for your application


    def quaternion_to_transformation_matrix(self, w, x, y, z, tx, ty, tz):
        rotation = R.from_quat([x, y, z, w]).as_matrix()
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation
        transformation_matrix[:3, 3] = [tx, ty, tz]
        return transformation_matrix

    def realtime_point(self):
        self.capture.readdata()
        frame_data = self.capture.get_quanternion_data()
        print("start-realtime")
        # if not frame_data or len(frame_data) < 2:
        #     print("Incomplete frame data received.")
        #     return None
        print(frame_data)
        try:
            # Assuming frame_data is structured as [(w1, x1, y1, z1, tx1, ty1, tz1), (w2, x2, y2, z2, tx2, ty2, tz2)]
            tool_data= frame_data[0]
            ref_data = frame_data[1]
            if tool_data[6]==0 or ref_data[6]==0:
                return
        # Extract quaternion and translation vectors
            w1, x1, y1, z1, t_x1, t_y1, t_z1 = tool_data
            w2, x2, y2, z2, t_x2, t_y2, t_z2 = ref_data

        # Compute homogenous transformation matrices
            Data_point_tool_h = self.quaternion_to_transformation_matrix(w1, x1, y1, z1, t_x1, t_y1, t_z1)
            Data_point_ref_h = self.quaternion_to_transformation_matrix(w2, x2, y2, z2, t_x2, t_y2, t_z2)

        # Combine tool tip with tool homogenous matrix
            Data_point_tool_tip_h = np.dot(Data_point_tool_h, self.Homo_tip)

        # Compute the inverse of the reference frame matrix and the relative position
            Data_point_ref_inv_h = np.linalg.inv(Data_point_ref_h)
            Data_point_tool_tip_h_relative = np.dot(Data_point_ref_inv_h, Data_point_tool_tip_h)
        
            relative_position_x = Data_point_tool_tip_h_relative[0, 3]
            relative_position_y = Data_point_tool_tip_h_relative[1, 3]
            relative_position_z = Data_point_tool_tip_h_relative[2, 3]

            return relative_position_x, relative_position_y, relative_position_z
        except Exception as e:
            print(f"Error processing frame data: {e}")
            return None

    def reset_tracking(self):
        if self.capture:
            self.capture.disconnect()  # Assuming disconnect method properly stops and closes the connection
        self.ax.clear()
        self.coordinates = []
        self.canvas.draw()
        self.is_tracking = False
        self.start_tracking()

    def complete_tracking(self):
        if self.capture:
            print("Stopping tracking...")
            self.capture.disconnect()
        if self.coordinates:
            full_path = "material\gather_point.pcd" 
            self.save_pcd_file(full_path, self.coordinates)
        self.is_tracking = False
        self.coordinates = []
        self.update_idletasks()
        time.sleep(2)  # Short delay to allow UI updates
        # Running ICP process
        
        self.run_icp_and_server()

    def run_icp_and_server(self):
        icp_script_path = r"C:\Users\BARTLAB\Desktop\Final (2)\Final\Iterative_closestpoint.py"
        server_script_path = r"C:\Users\BARTLAB\Desktop\Final (2)\Final\server.py"

        # Run ICP process
        try:
            subprocess.run(["python", icp_script_path], check=True)
            print("ICP processing complete. Starting server...")
        except subprocess.CalledProcessError as e:
            print(f"ICP script error: {e}")
            return

       # Start server process
        self.server_process = subprocess.Popen(["python", server_script_path])
        print("Server is running...")




    def save_pcd_file(self, file_path, coordinates):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(coordinates)
        o3d.io.write_point_cloud(file_path, pcd)
        print(f"Point cloud data saved to: {file_path}")

    def update_plot(self):
        while not self.update_queue.empty():
            data_point = self.update_queue.get()
            if data_point:  # Ensure data_point is not None
                self.coordinates.append(data_point)
                self.ax.scatter(*data_point, color='red')
        self.canvas.draw()
        self.after(100, self.update_plot)  # Schedule next update

    def change_appearance_mode(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)

    def run(self):
        self.after(100, self.update_plot)  # Start the periodic update of the plot
        self.mainloop()

    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            if self.server_process and self.server_process.poll() is None:
                self.server_process.terminate()  # Attempt to terminate the server process
                self.server_process.wait()  # Wait for the process to terminate
            self.destroy()

if __name__ == "__main__":
    app = TrackingApplication()
    app.run()