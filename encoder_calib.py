from cmath import pi
import csv
import odrive
import numpy as np
from odrive.enums import *
import datetime
import time
import math
import numpy as np
from matplotlib import pyplot as plt
print('Finding ODrive')
my_drive = odrive.find_any()
motor = my_drive.axis0.motor
axis = my_drive.axis0
ctrl = axis.controller
print('ODrive Connected')

print("Starting closed loop control...")
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

print("Starting velocity control...")
my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
desired = []
dotphi = [] 
timerx = [] 
t_start = my_drive.system_stats.uptime # start time of trial
duration = 30 
P = 0.01
R = 0.009 
q = 0.005
w = 3.141
t_prev = 0
t_prev1 = 0
amp = 0.3
vel_est1 = (amp*w)
my_drive.axis0.encoder.set_linear_count(0)
phi_prev = 6.28*my_drive.axis0.encoder.pos_estimate
phivel_prev = 6.28*my_drive.axis0.encoder.vel_estimate
# Trajectory generation 
while(True):
    # Current Time
    time_x = (my_drive.system_stats.uptime - t_start)/1000 # current time in seconds
    if(time_x > duration):
        break
    vel_setpoint = round((amp*w*math.cos(w*time_x)),3)
    my_drive.axis0.controller.input_vel = vel_setpoint
    phivel_mea = round(my_drive.axis0.encoder.vel_estimate_counts,3)
    # phi_mea = round(6.28*my_drive.axis0.encoder.pos_estimate,3)
    # nowtime = my_drive.system_stats.uptime
    # dt = tdiff
    # if(dt == 0):
    #     phivel_mea = phivel_prev
    # else:
    #     dphi = phi_mea - phi_prev
    #     phivel_mea = dphi/dt
    # previous_estimate = vel_est1
    # KG = P/(P+R)
    # vel_est1 = vel_est1 + KG*(phivel_mea - vel_est1)    
    # P = (1-KG)*P + q*abs(previous_estimate - vel_est1)
    timerx.append(time_x)
    dotphi.append(phivel_mea)
    desired.append(vel_setpoint)
my_drive.axis0.controller.input_vel = 0 #To stop the motor at the end of the trial
my_drive.axis0.requested_state = AXIS_STATE_IDLE #to put the motor in idle state at the end of rotation

# Velocity Plot
fig1 = plt.figure()
plt.plot(timerx,desired,label='desired')
plt.plot(timerx,dotphi,label='actual')
plt.xlabel("Time in seconds")
plt.ylabel("Angular velocity in radians per second")
plt.legend()
plt.show()