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

# Arrays to save trajectory
theta_d = [] 
dottheta_d = []
ddottheta_d = []
timerx = [] 

# Trajectory parameters
w = 3.141 #Frequency of sine wave4
amp = 0.3
duration = 40 
t_start = my_drive.system_stats.uptime # start time of trial
t_prev = 0 
while(True):
    # Current Time
    time_x = (my_drive.system_stats.uptime - t_start)/1000 # current time in seconds
    tdiff = (time_x*1000) - t_prev
    if(time_x > duration):
        break
    if(tdiff >= 24):
        if(time_x < 10):
            theta_setpoint = round(amp*math.sin(w*time_x),3)
            dottheta_setpoint = round(amp*w*math.cos(w*time_x),3)
            ddottheta_setpoint = round(-amp*math.pow(w,2.0)*math.sin(w*time_x),3)
            theta_d.append(theta_setpoint)
            dottheta_d.append(dottheta_setpoint)
            ddottheta_d.append(ddottheta_setpoint)
        elif(time_x >= 10 and time_x < 20):  
            t1 = time_x - 10
            theta_setpoint = 0.03*t1
            dottheta_setpoint = 0.03
            ddottheta_setpoint = 0.0
            theta_d.append(theta_setpoint)
            dottheta_d.append(dottheta_setpoint)
            ddottheta_d.append(ddottheta_setpoint)        
        elif(time_x >= 20 and time_x < 30):
            theta_setpoint = 0.3
            dottheta_setpoint = 0.0
            ddottheta_setpoint = 0.0
            theta_d.append(theta_setpoint)
            dottheta_d.append(dottheta_setpoint)
            ddottheta_d.append(ddottheta_setpoint)   
        else:
            t2 = time_x - 30
            theta_setpoint = -0.03*t2 + 0.3
            dottheta_setpoint = -0.03
            ddottheta_setpoint = 0.0
            theta_d.append(theta_setpoint)
            dottheta_d.append(dottheta_setpoint)
            ddottheta_d.append(ddottheta_setpoint)  
        print(tdiff) 
        timerx.append(time_x)
        t_prev = time_x*1000

print("Size of theta_d ",len(theta_d))
print("Size of dottheta_d ",len(dottheta_d))
print("Size of ddottheta_d ",len(ddottheta_d))
# Position Plot
fig1 = plt.figure()
plt.plot(timerx,theta_d,label='desired')
plt.xlabel("Time in seconds")
plt.ylabel("Theta in radians")
plt.legend()

# Velocity Plot
fig2 = plt.figure()
plt.plot(timerx,dottheta_d,label='desired')
plt.xlabel("Time in seconds")
plt.ylabel("Angular velocity in radians per second")
plt.legend()    

# Acceleration Plot
fig3 = plt.figure()
plt.plot(timerx,ddottheta_d,label='desired')
plt.xlabel("Time in seconds")
plt.ylabel("Angular acceleration in radians per second squared")
plt.legend()   
plt.show()
print('done')
quit()