from cmath import pi
import odrive
import numpy as np
from odrive.enums import *
import time
import math
import numpy as np
from matplotlib import pyplot as plt
my_drive = odrive.find_any()
motor = my_drive.axis0.motor 
axis = my_drive.axis0 
ctrl = axis.controller  #variables
# print("Starting calibration...")
# my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# time.sleep(130)
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
my_drive.axis0.controller.input_pos = -2
while True:
    print(my_drive.axis0.encoder.pos_estimate)
    if(my_drive.axis0.encoder.pos_estimate < -3):
        break   