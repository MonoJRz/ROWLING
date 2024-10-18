import os
import sys
import time


sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from xarm.wrapper import XArmAPI

########################################################
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', '192.168.1.231')
    except:
        ip = "192.168.1.231"

arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.clean_error()
arm.set_mode(0)
arm.set_state(0)
time.sleep(0.1)

last_digitals = [-1, -1]
while arm.connected and arm.error_code != 19 and arm.error_code != 28:
    code, digitals = arm.get_tgpio_digital()
    if code == 0:
        if digitals[0] == 1 and digitals[0] != last_digitals[0]:
            print('IO0 input high level')
        
        if digitals[0] == 0 and digitals[0] != last_digitals[0]:
            print('IO0 input low level')
       