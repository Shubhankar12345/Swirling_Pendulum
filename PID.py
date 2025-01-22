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
my_drive = odrive.find_any()
motor = my_drive.axis0.motor
axis = my_drive.axis0
ctrl = axis.controller

print("Starting closed loop control...")
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("Starting torque control...")
my_drive.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
torque_input_UL = 0.83
torque_input_LL  = -0.83
prev_error = 0
prev_measurement = 0
Ts = 0.005
Tau = 10000
K_p = 0.23
K_i = 0.044
K_i = 0 
K_d = 0
t_s = 0.01
# t = [0 + t_s*interval for interval in range(1001)]
t = []
pos_plot = []
i = 0
yi = 0
yf = math.pi
tf = 30 # for unstable equilibrium transfer
# tf = 15 # for stable equilibrium transfer
duration = tf
# for j in t:
#     val = yi+(yf-yi)*(10*math.pow((j/duration),3) - 15*math.pow((j/duration),4) + 6*math.pow((j/duration),5))
#     pos_plot.append(val)
previous_time = my_drive.system_stats.uptime
phi_actual = []
# Iq_measured_current = []
Iq_setpoint_current = []
current_time = 0
# count = 0
# counts = []
I_input = 0
D_input = 0
val = 0
print('-----------------')
error_angle = 0
t_start = my_drive.system_stats.uptime
torque = []
theta_actual = []
phi  = 0
while(True):
    # current_time = my_drive.system_stats.uptime
    # # if(current_time - previous_time < 1000):
    # if(current_time - previous_time > 1000 and current_time - previous_time < 1010):
    #     print(count)
    #     counts.append(count)
    #     count = 0
    #     previous_time = current_time
    # count = count + 1
    time = (my_drive.system_stats.uptime - t_start)/1000
    val = yi+(yf-yi)*(10*math.pow((time/duration), 3) - 15*math.pow((time/duration), 4) + 6*math.pow((time/duration), 5))
    if time > tf:
        val = yf
    phi = my_drive.axis0.encoder.pos_estimate*2*math.pi #current measurement
    error_angle = val - phi #error in phi
    # PID
    # if(time>10):
    #     K_d = 0.01
    P_input = K_p*error_angle
    I_input = I_input + (K_i*Ts*(error_angle + prev_error))/2
    D_input = (2*K_d/(2*Tau+Ts))*(error_angle - prev_error) + ((2*Tau-Ts)/(2*Tau+Ts))*D_input # Derivative in form of difference equation applied on the error
    if(torque_input_UL > P_input): #Integrator dynamic clamping
        I_input_UL = torque_input_UL - P_input
    else:
        I_input_UL = 0
    if(torque_input_LL < P_input):
        I_input_LL = torque_input_LL - P_input
    else:
        I_input_LL = 0
    if(I_input > I_input_UL):
        I_input = I_input_UL
    elif(I_input < I_input_LL):
        I_input = I_input_LL
    torque_input = (P_input + I_input + D_input)
    if(torque_input > torque_input_UL): #PID saturation limit
        torque_input = torque_input_UL
    elif(torque_input < torque_input_LL):
        torque_input = torque_input_LL
    # print(torque_input)
    my_drive.axis0.controller.input_torque = torque_input
    prev_error = error_angle
    prev_measurement = phi
    Iq_setpoint_current.append(my_drive.axis0.motor.current_control.Iq_setpoint)
    theta_actual.append(-my_drive.axis1.encoder.pos_estimate*2*math.pi)
    # Iq_measured_current.append(my_drive.axis0.motor.current_control.Iq_measured)
    phi_actual.append(my_drive.axis0.encoder.pos_estimate*2*math.pi)
    pos_plot.append(val)
    torque.append(torque_input)
    t.append(time)
    if(time > tf or phi>2*math.pi):
        break
    # if (time > 20 or theta > 2 * math.pi):
    #     break

if(abs(my_drive.axis0.encoder.pos_estimate)>1): # Stop the motor if the if one revolution has been completed to prevent the system from becoming unstable
    my_drive.axis0.controller.input_torque = 0
my_drive.axis0.controller.input_torque = 0 #To stop the motor at the end of the trial
my_drive.axis0.requested_state = AXIS_STATE_IDLE #to put the motor in idle state at the end of rotation
plt.plot(t, pos_plot,label = 'desired')
plt.plot(t,phi_actual,label = 'actual')
plt.xlabel("time in seconds")
plt.ylabel("phi in radians")
plt.legend()
fig = plt.figure()
fig = plt.plot(t,theta_actual,label="theta")
plt.xlabel("time in seconds")
plt.ylabel("theta in radians")
plt.legend()
fig = plt.figure()
fig = plt.plot(t,torque,label='control i/p')
plt.xlabel("time in seconds")
plt.ylabel("torque in Nm")
plt.legend()
# x = str(datetime.datetime.now())
# x = x.replace(":","-")
# y = ('Swirling pendulum minimum jerk trajectory tracking for stable equilibrium to stable equilibrium transfer -{}.csv').format(x)
# with open(y, 'w', newline='') as f: # Storing the data in CSV format
#     thewriter = csv.writer(f)
#     thewriter.writerow(['time','pos_plot','phi_actual','theta_actual', 'Torque', 'setpoint current', 'kp', 'ki', 'kd', 'Tau'])
#     for i in range(0, len(t)):
#         thewriter.writerow([t[i],pos_plot[i],phi_actual[i],theta_actual[i],torque[i], Iq_setpoint_current[i], K_p, K_i, K_d, Tau])
#Scounts.sort()
# tv = int(0.26*len(counts))
# counts = counts[tv:]
# # print(counts)
# # counts = counts[:len(counts)-tv]
# print("counts per second ", (sum(counts))/len(counts))
plt.show()
quit()

