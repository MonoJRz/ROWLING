import socket
import numpy as np

def receive_position_data_from_server(server_address, server_port):
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connect the socket to the server
    server = (server_address, server_port)
    print(f"Connecting to {server_address} port {server_port}")
    sock.connect(server)

    try:
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
                else:
                    print("Incomplete data received.")

    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        print("Closing connection")
        sock.close()

if __name__ == "__main__":
    HOST, PORT = "127.0.0.1", 12345  # Change server_address_here to your server's address
    receive_position_data_from_server(HOST, PORT)
