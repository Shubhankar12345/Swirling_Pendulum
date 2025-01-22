from cmath import pi
import odrive
import numpy as np
from odrive.enums import *
import time
import statistics
import math
import random as rnd
from matplotlib import pyplot as plt
my_drive = odrive.find_any()
set_previous = my_drive.axis0.encoder.pos_estimate*2*math.pi
motor = my_drive.axis0.motor; axis = my_drive.axis0; ctrl = axis.controller  #variables
# my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# # print("Starting calibration...")
# # time.sleep(130)
print("starting closed loop control...")
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
# theta_d1 = np.linspace(0,20*math.pi, 1000)
# theta_d = np.array([theta_d1])
# val = []
# val1 = []
# s = theta_d.shape
# for i in range(0, s[0]):
#     for j in range(0, s[1]):
#         x1 = math.sin(theta_d[i][j])
#         y1 = math.cos(theta_d[i][j])
#         val.append(x1)
#         val1.append(y1)
# val = np.array([val])
# val1 = np.array([val1])
# t1 = []
# t2 = []
# t3 = []
# for i in range(0, s[0]):
#     for j in range(0, s[1]):
#         t1.append(theta_d[i][j])
#         t2.append(val[i][j])
#         t3.append(val1[i][j])
# t = []
theta_actual = []
vel_actual = []
vel_meas = []
vel_des = []
e_est = 0.1
# count = 0
# counts = []
# previous_time = my_drive.system_stats.uptime
est = 1 #initial estimate of velocity
P = 0.5
Q = math.exp(-6)
R = 0.8
flag = True
est1 = []
duration = 10
w = 1
q = 0.00001
desired_pos = []
timerx = []
time_x = 0
vel_meas1 = []
vel_lp = []
Iqsp = []
Id = [] 
vel_mea = est
t_start = my_drive.system_stats.uptime
vel_filtered_output = 0
vel_mea_previous = 0
vel_filtered_output_previous = 0
while(time_x <= duration):
    # current_time = my_drive.system_stats.uptime
    # # if(current_time - previous_time < 1000):
    # if(current_time - previous_time > 1000 and current_time - previous_time < 1010):
    #     print(count)
    #     counts.append(count)
    #     count = 0
    #     previous_time = current_time
    # count = count + 1
    time_x = (my_drive.system_stats.uptime - t_start)/1000
    setpoint = math.sin(w*time_x)
    t1 = my_drive.system_stats.uptime 
    my_drive.axis0.controller.input_pos = setpoint
    time.sleep(0.0001)
    # dt = (my_drive.system_stats.uptime-t1)/1000 #dt to determine the instantaneous velocity
    current_pos = my_drive.axis0.encoder.pos_estimate
    # error = math.cos(w*time_x) - my_drive.axis0.encoder.vel_estimate # measurement error. Will append this in a list and calculate its variance to give measurement uncertainity.
    # if(dt == 0):
    #     vel_mea = est
    #     vel_mea_previous = vel_mea
    # else:
    #     vel_mea = (current_pos-set_previous)/dt
    #     vel_filtered_output = 0.9359*vel_filtered_output_previous + 0.064*vel_mea_previous
    # set_previous = current_pos
    # est1.append(my_drive.axis0.encoder.vel_estimate)
    # previous = est
    # P = P + Q
    # KG = P/(P+R)
    # est = est + KG*(vel_filtered_output - est)
    # P = (1-KG)*P + q*abs(previous - est)
    desired_pos.append(setpoint)
    timerx.append(time_x)
    vel_lp.append(vel_filtered_output)
    # vel_meas.append(error)
    # vel_meas1.append(vel_mea)
    # vel_des.append(math.cos(w*time_x))
    Iqsp.append(my_drive.axis0.motor.current_control.Iq_setpoint)
    Id.append(my_drive.axis0.motor.current_control.Iq_measured)
    theta_actual.append(current_pos)
    # vel_filtered_output_previous = vel_filtered_output
    # vel_mea_previous = vel_mea
# tv = int(0.114*len(counts))
# counts = counts[tv:]
# print("counts array length", len(counts))
# print("counts array is ",counts)
# # counts = counts[:len(counts)-tv]
# print("counts per second ", (sum(counts))/len(counts))
# print(statistics.variance(vel_meas))
my_drive.axis0.controller.input_pos = 0
fig = plt.figure()
plt.plot(timerx,desired_pos,label='desired')
plt.plot(timerx,theta_actual,label='actual')
plt.xlabel("time in seconds")
plt.ylabel("angular displacement in no. of turns")
plt.legend()
fig1 = plt.figure()
# plt.plot(timerx,vel_des,label='desired')
# plt.plot(t1,vel_actual,label='actual')
plt.plot(timerx,est1,label='actual')
plt.xlabel("time in seconds")
plt.ylabel("angular velocity in turns per sconds")
plt.legend()
fig2 = plt.figure()
plt.plot(timerx,Iqsp,label='desired current')
plt.plot(timerx,Id,label='actual current')
plt.xlabel("Time in seconds")
plt.ylabel("Current in A")
plt.legend()
# fig2 = plt.figure()
# plt.plot(t1,vel_meas,label='odrive estimate')
# plt.xlabel("time in seconds")
# plt.ylabel("angular velocity in radians per second")
# plt.legend()
plt.show()
quit()

