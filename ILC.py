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
torque_input_UL = 0.22
torque_input_LL  = -0.22
yi = 0
yf = (math.pi)/2
trials = 2
trial_error = []
initial_phi_error = yi - my_drive.axis0.encoder.pos_estimate*2*math.pi
previous_control_input = []
previous_trial_error = []
previous_torque_ip = 0 
gamma = 0.003 # Learning gain
duration = 30
timerx = []
phi_actual = []
phi_desired = []
theta_actual = []
trial_data = {}
count = 0
time_trial = 0
# trial_data = np.empty((trials,1),dtype='float')
while(trials > 0):
    t_start = my_drive.system_stats.uptime
    if(trials == 2):
        while(time_trial <= duration):
            time_trial = (my_drive.system_stats.uptime - t_start)/1000
            phi_setpoint = yi+(yf-yi)*(10*math.pow((time_trial/duration), 3) - 15*math.pow((time_trial/duration), 4) + 6*math.pow((time_trial/duration), 5))
            torque_ip = (previous_torque_ip + gamma*initial_phi_error)*0.5
            if(torque_ip > torque_input_UL):
                torque_ip = torque_input_UL
            if(torque_ip < torque_input_LL):
                torque_ip = torque_input_LL
            my_drive.axis0.controller.input_torque = torque_ip
            # time.sleep(0.001)
            phi_current = my_drive.axis0.encoder.pos_estimate*2*math.pi
            error_phi_current = phi_setpoint - phi_current
            initial_phi_error = error_phi_current
            theta_current = my_drive.axis1.encoder.pos_estimate*2*math.pi
            previous_torque_ip = torque_ip
            phi_actual.append(phi_current)
            phi_desired.append(phi_setpoint)
            theta_actual.append(theta_current)
            previous_control_input.append(torque_ip)
            previous_trial_error.append(error_phi_current)
            timerx.append(time_trial)
        # data = (phi_actual,phi_desired,theta_actual,previous_control_input,previous_trial_error)
        # trial_data["Trial"+str(count)] = data
    # if(trials == 1):
    #     for i in range(0,len(timerx)):
    #         time_trial = (my_drive.system_stats.uptime - t_start)/1000
    #         phi_setpoint = yi+(yf-yi)*(10*math.pow((time_trial/duration), 3) - 15*math.pow((time_trial/duration), 4) + 6*math.pow((time_trial/duration), 5))
    #         torque_ip = previous_control_input[i] + gamma*previous_trial_error[i]
    #         if(torque_ip > torque_input_UL):
    #             torque_ip = torque_input_UL
    #         if(torque_ip < torque_input_LL):
    #             torque_ip = torque_input_LL
    #         my_drive.axis0.controller.input_torque = torque_ip
    #         # time.sleep(0.001)
    #         phi_current = my_drive.axis0.encoder.pos_estimate*2*math.pi
    #         error_phi_current = phi_setpoint - phi_current            
    #         theta_current = my_drive.axis1.encoder.pos_estimate*2*math.pi
    #         phi_actual[i] = phi_current
    #         theta_actual[i] = theta_current
    #         previous_control_input[i] = torque_ip            
    #         previous_trial_error[i] = error_phi_current
    else:
        for i in range(0,len(timerx)):
            time_trial = (my_drive.system_stats.uptime - t_start)/1000
            phi_setpoint = yi+(yf-yi)*(10*math.pow((time_trial/duration), 3) - 15*math.pow((time_trial/duration), 4) + 6*math.pow((time_trial/duration), 5))
            torque_ip = previous_control_input[i] + gamma*previous_trial_error[i]
            if(torque_ip > torque_input_UL):
                torque_ip = torque_input_UL
            if(torque_ip < torque_input_LL):
                torque_ip = torque_input_LL
            my_drive.axis0.controller.input_torque = torque_ip
            # time.sleep(0.001)
            phi_current = my_drive.axis0.encoder.pos_estimate*2*math.pi
            error_phi_current = phi_setpoint - phi_current            
            theta_current = my_drive.axis1.encoder.pos_estimate*2*math.pi
            phi_actual[i] = phi_current
            theta_actual[i] = theta_current
            previous_control_input[i] = torque_ip            
            previous_trial_error[i] = error_phi_current

    # try:
    #     print(trial_data["Trial0"][-1])
    # except:
    #     pass

    data1 = [phi_actual.copy(),phi_desired.copy(),theta_actual.copy(),previous_control_input.copy(),previous_trial_error.copy()]
    trial_data["Trial"+str(count)] = data1.copy()
    # print(trial_data["Trial0"][-1])
    # print()
    # print()
    trials = trials-1
    count = count+1
my_drive.axis0.controller.input_torque = 0 #To stop the motor at the end of all trials
my_drive.axis0.requested_state = AXIS_STATE_IDLE #To stop the motor at the end of all trials
# print(trial_data.keys())
x = trial_data['Trial0'][0]
print(x)
print()
# print()
# y = trial_data['Trial1']
# print(y)
fig1 = plt.figure()
flag = True
plt.plot(timerx,trial_data["Trial0"][0])
# for key in trial_data.keys():   
#     values = trial_data[key]
#     # (phi_actual_1,phi_desired_1,theta_actual_1,previous_control_input_1,previous_trial_error_1)=values
#     if(flag):
#         plt.plot(timerx,values[1],linewidth=1, label='desired')
#         flag = False
#     plt.plot(timerx, values[0], linewidth=1, label=key)
plt.xlabel('Time in seconds')
plt.ylabel('Phi in radians')
plt.legend()
plt.show()
# x = str(datetime.datetime.now())
# x = x.replace(":","-")
# y = ('Swirling pendulum minimum jerk trajectory ILC -{}.csv').format(x)
# with open(y, 'w', newline='') as f: # Storing the data in CSV format
#     thewriter = csv.writer(f)
#     thewriter.writerow(['time','phi_actual','phi_desired','theta_actual', 'Torque', 'Error'])
#     for key,value in trial_data.items():
#         thewriter.writerow([key,value])
#     for key in trial_data.keys():
#         values = trial_data[key]
#         for i in range(0,len(timerx)):
#             thewriter.writerow([timerx[i],values[0][i],values[1][i],values[2][i],values[3][i],values[4][i]])
# values = trial_data['Trial1']
# print(type(values))
# time.sleep(5)