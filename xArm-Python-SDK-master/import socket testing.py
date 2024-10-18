import socket
import concurrent.futures
from xarm import version
from xarm.wrapper import XArmAPI
import ast
import time

# Define the server address and port to listen on
host = '127.0.0.1'  # Replace with your server's IP address
port = 12345  # Replace with the port number used in your C++ program

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the server address and port
server_socket.bind((host, port))

# Listen for incoming connections (1 connection at a time)
server_socket.listen(1)

class RobotMain(object):
    def __init__(self, robot, **kwargs):
        self._arm = robot
        self._tcp_speed = 300  # Adjusted speed
        self._tcp_acc = 800  # Adjusted acceleration
        self._angle_speed = 400  # Adjusted angular speed
        self._angle_acc = 800  # Adjusted angular acceleration
        self._vars = {}
        self._funcs = {}
        self._robot_init()

    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)

    def move_to_position(self, target_position):
        try:
            code = self._arm.set_position(*target_position, speed=self._tcp_speed, mvacc=self._tcp_acc, radius=5000.0, wait=True)
        except Exception as e:
            print("Exception while moving to position:", str(e))

def process_data(data):
    try:
        # Safely evaluate the data as a literal to extract the numeric values
        # Remove any additional characters and split by comma
        data = data.strip("()").replace(" ", "")
        values = ast.literal_eval(data)
        return values
    except (ValueError, SyntaxError):
        return None

def client_handler(client_socket, robot_main):
    while True:
        data = client_socket.recv(4096)
        if not data:
            break

        received_data = data.decode('utf-8')
        values = process_data(received_data)

        if values is not None:
            start_time = time.time()
            robot_main.move_to_position(values)
            end_time = time.time()
            elapsed_time = end_time - start_time
            print(f"Task completed in {elapsed_time:.2f} seconds")
        
        else:
            print("Error interpreting data:", received_data)

    # The client_socket is not closed here to allow continuous data reception and robot movement

if __name__ == '__main__':
    print('xArm-Python-SDK Version:', version.__version__)

    arm = XArmAPI('192.168.1.205', baud_checkset=False)
    robot_main = RobotMain(arm)

    with concurrent.futures.ThreadPoolExecutor() as executor:
        while True:
            client_socket, client_address = server_socket.accept()
            print(f"Accepted connection from {client_address}")

            # Use submit to asynchronously execute client_handler
            future = executor.submit(client_handler, client_socket, robot_main)
