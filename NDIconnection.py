import serial
import time
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

class Capture:
    file_path1 = "material\8700340.rom"
    file_path2 = "material\8700339.rom"
    file_path3 = "material\8700338.rom"
    def __init__(self):
        
        self.my_serial_port = None
        self.homo_data = None
        self.quanternion_data = None
    
    
    def connect(self):
        try:
            self.my_serial_port = serial.Serial(
                                                port='COM6',  # Serial port to use (replace 'COM7' with your port)
                                                baudrate=9600,  # Baud rate
                                                parity=serial.PARITY_NONE,  # Parity bit
                                                stopbits=serial.STOPBITS_ONE,  # Stop bits
                                                bytesize=serial.EIGHTBITS,  # Data bits
                                                timeout=1,  # Read timeout in seconds
                                                rtscts=False,  # Disable hardware (RTS/CTS) flow control
                                                xonxoff=False,  # Disable software flow control
                                                dsrdtr=False , # Disable hardware (DSR/DTR) flow control
                                                
                                            )

            
            print("connected")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}", file=sys.stderr)
            sys.exit(1)

    def disconnect(self):
        if self.my_serial_port and self.my_serial_port.is_open:
            self.my_serial_port.close()
            print("disconnected")


    def init_polaris(self):
        print("Sending commands to initialize Polaris...")
        
        # Implementation for initializing Polaris
        # print("RESET 0\r")
        self.my_serial_port.write(b"RESET 0\r")
        time.sleep(2)
        # self.read_com()



        
        self.my_serial_port.write(b"COMM 50000\r")
        time.sleep(1)
        # self.read_com()

        self.my_serial_port.close()
        time.sleep(1)
        self.my_serial_port = serial.Serial('COM6', 
                                                baudrate=115200, 
                                                parity=serial.PARITY_NONE,  # Parity bit
                                                stopbits=serial.STOPBITS_ONE,  # Stop bits
                                                bytesize=serial.EIGHTBITS,  # Data bits
                                                timeout=2,  # Read timeout in seconds
                                                rtscts=False,  # Disable hardware (RTS/CTS) flow control
                                                xonxoff=False,  # Disable software flow control
                                                dsrdtr=False,  # Disable hardware (DSR/DTR) flow control
                                                writeTimeout=2
                                             )
        
        
        
        self.my_serial_port.write(b"INIT \r")
        time.sleep(1)
        # self.read_com()

        self.send_config(self.file_path1, 1)
        self.send_config(self.file_path2, 2)
        self.send_config(self.file_path3, 3)

        self.my_serial_port.write(b"PHSR 02\r")
        time.sleep(0.01)
        # self.read_com()

        self.my_serial_port.write(b"PINIT 01\r")
        time.sleep(0.01)
        # self.read_com()

        self.my_serial_port.write(b"PINIT 02\r")
        time.sleep(0.01)
        # self.read_com()

        self.my_serial_port.write(b"PINIT 03\r")
        time.sleep(0.01)
        # self.read_com()

        self.my_serial_port.write(b"PHSR 03\r")
        time.sleep(0.01)
        # self.read_com()

        self.my_serial_port.write(b"PENA 01D\r")
        time.sleep(0.01)
        # self.read_com()

        self.my_serial_port.write(b"PENA 02D\r")
        time.sleep(0.01)
        # self.read_com()

        self.my_serial_port.write(b"PENA 03D\r")
        time.sleep(0.01)
        # self.read_com()

        self.my_serial_port.write(b"TSTART \r")
        time.sleep(0.01)
        # self.read_com()


        time.sleep(1)

        
        
    def read_file(self,file_path):
        with open(file_path, 'rb') as file_stream:
            buffer = file_stream.read()
        return buffer

    def read_com(self):
        if self.my_serial_port.in_waiting > 0:  # Check if there is data available to read
            message = self.my_serial_port.readline().decode('utf-8').strip()
            print(message)
            return True  # Indicates that a message was read
        else:
            print("No message available on the serial port.")
            return False  # Indicates that no message was available to read

    def send_config(self, file_path, id):

        buff = bytearray(768)
        buff[0:768] = self.read_file(file_path)
        command = '-'.join(['{:02X}'.format(b) for b in buff])
        hexValuesSplit = command.split('-')
        send = ""
        hexOutput = ""
        # print("PHSR 01\r")
        self.my_serial_port.write(b"PHSR 01\r")
        time.sleep(0.05)
        # self.read_com()

        # print("PHRQ ********1****\r")
        self.my_serial_port.write(b"PHRQ ********01****\r")
        time.sleep(0.05)
        # self.read_com()
    
        


        i=0
        while i < 768:
            send = ""
            if (id == 1):
                hexOutput = f"PVWR 01{i:04X}"
            elif (id == 2):
                        hexOutput = f"PVWR 02{i:04X}"
            elif (id == 3):
                        hexOutput = f"PVWR 03{i:04X}"

            for j in range(64):
                if (i <= 751):
                    send = send + hexValuesSplit[i]
                    i += 1
                else:
                    send = send + "00"
                    i += 1
            sending = hexOutput + send
            # print(sending)
            self.my_serial_port.write((sending + "\r").encode())
            time.sleep(0.05)
            # self.read_com()
    
    def quaternion_to_transformation_matrix(self,w, x, y, z, tx, ty, tz):
    # Placeholder implementation - replace with actual function
        rotation = R.from_quat([x, y, z, w]).as_matrix()
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation
        transformation_matrix[:3, 3] = [tx, ty, tz]
        transformation_matrix_result=np.array([transformation_matrix])
        return transformation_matrix_result
    
    def get_homo_data(self):
        return self.homo_data
    
    def get_quanternion_data(self):
        return self.quanternion_data
    
    def readdata(self):
        data = ""
        M1 = []  # An empty list, equivalent to an empty char array
        M2 = []  # Another empty list
        M3 = []  # And another
        m1x = m1y = m1z = m1e = 0
        M2x = M2y = M2z = M2e = 0
        M3x = M3y = M3z = M3e = 0
        m1r0 = M2r0 = M3r0 = 1
        m1rx = M2rx = M3rx = 0
        m1ry = M2ry = M3ry = 0
        m1rz = M2rz = M3rz = 0
        
        self.my_serial_port.write(b"TX 0801\r")
        
        time.sleep(0.035)

        data = self.my_serial_port.read(self.my_serial_port.inWaiting()).decode('utf-8')
        
        try:
            data_split = str(data).split('\n')
            M1 = (data_split[0])
            M2 = (data_split[1])
            M3 = (data_split[2])
            
            if len(M1)==71 or len(M1)==553:
                m1r0 = int(M1[4:10])/10000
                m1rx = int(M1[10:16])/10000
                m1ry = int(M1[16:22])/10000
                m1rz = int(M1[22:28])/10000
                m1x  = int(M1[28:35])/100
                m1y  = int(M1[35:42])/100
                m1z  = int(M1[42:49])/100
            
            if len(M2)==69: 
                M2r0 = int(M2[2:8])/10000
                M2rx = int(M2[8:14])/10000
                M2ry = int(M2[14:20])/10000
                M2rz = int(M2[20:26])/10000
                M2x  = int(M2[26:33])/100
                M2y  = int(M2[33:40])/100
                M2z  = int(M2[40:47])/100

            
            if len(M3)==69:
                M3r0 = int(M3[2:8])/10000
                M3rx = int(M3[8:14])/10000
                M3ry = int(M3[14:20])/10000
                M3rz = int(M3[20:26])/10000
                M3x  = int(M3[26:33])/100
                M3y  = int(M3[33:40])/100
                M3z  = int(M3[40:47])/100
            
            # if len(M1)==553:
            #      if M1[4:10]
######################################################mark2##############################
            quandata1 = np.array([m1r0, m1rx, m1ry, m1rz, m1x, m1y, m1z])
            quandata2 = np.array([M2r0, M2rx, M2ry, M2rz, M2x, M2y, M2z])
            quandata3 = np.array([M3r0, M3rx, M3ry, M3rz, M3x, M3y, M3z])
            # Assuming CvtQuatToRotationMatrix and Homogenous functions are defined elsewhere
            HomoM1 = self.quaternion_to_transformation_matrix(m1r0, m1rx, m1ry, m1rz, m1x, m1y, m1z)
            HomoM2 = self.quaternion_to_transformation_matrix(M2r0, M2rx, M2ry, M2rz, M2x, M2y, M2z)
            HomoM3 = self.quaternion_to_transformation_matrix(M3r0, M3rx, M3ry, M3rz, M3x, M3y, M3z)
            
            self.quanternion_data=np.vstack((quandata1,quandata2,quandata3))
            self.homo_data=np.vstack((HomoM1,HomoM2,HomoM3))
            
            return  self.homo_data
        except Exception as e:
            print(f"Error: {e}")
            # If you're using a GUI, replace the print statement with a dialog message box or appropriate error handling
            return  None
# capture= Capture()
# capture.connect()
# capture.init_polaris()
# while True:
#      capture.readdata()
#      time.sleep(0.05)
