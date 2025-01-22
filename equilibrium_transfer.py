from cmath import pi
import csv
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
# time.sleep(150)
print("Starting closed loop control...")
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("Starting torque control...")
my_drive.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
theta_setpoint = 0.1899*2*math.pi
phi_setpoint = (math.pi)/2
theta_0 = my_drive.axis1.encoder.pos_estimate*2*math.pi
phi_0 = my_drive.axis0.encoder.pos_estimate*2*math.pi
print('theta_0 = '+str(theta_0))
print('phi_0 = '+str(phi_0))
flag = True
t0 = time.monotonic()
torque_input_UL = 1
torque_input_LL  = -1
prev_error = 0
prev_measurement = 0
Ts = 0.01
Tau = 0.1
K_p = 0.04
K_i = 0.02    
K_d = 0.03                                                                                                                                                                                                                                                                                                                                        
t = np.linspace(0,5,200)
t = np.array([t])
s = t.shape
pos = theta_setpoint*np.ones(s)
pos1 = phi_setpoint*np.ones(s)
pos_plot = []
pos_plot1 = []
timerx = []
for i in range(0, s[0]):
    for j in range(0, s[1]):
        timerx.append(t[i][j])
        pos_plot.append(pos[i][j])
        pos_plot1.append(pos1[i][j])
#print(pos_plot)
#print(timerx)
theta_actual = []
Iq_measured_current = []
Iq_setpoint_current = []
phi_actual = []
I_input = 0
D_input = 0
print('-----------------')
for i in range(0, s[1]):
    theta = (my_drive.axis1.encoder.pos_estimate)*2*math.pi#current theta measurement
    if(theta > 1): # To break the loop if system becomes unstable
        break
    error_angle = theta_setpoint - theta #error in phi
    P_input = K_p*error_angle
    I_input = I_input + (K_i*Ts*(error_angle + prev_error))/2
    D_input = (2*K_d/(2*Tau+Ts))*(error_angle - prev_error) + ((2*Tau-Ts)/(2*Tau+Ts))*D_input # Derivative in form of difference equation applied on the error
#    D_input = (2*K_d/(2*Tau+Ts))*(error_angle - prev_error) + ((2*Tau-Ts)/(2*Tau+Ts))*D_input # Derivative in form of difference equation applied on the measurement to prevent the derivative kick. Valid only if the setpoint is constant with respect to time
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
    torque_input = P_input + I_input + D_input
    if(torque_input > torque_input_UL): #PID saturation limit
        torque_input = torque_input_UL
    elif(torque_input < torque_input_LL):
        torque_input = torque_input_LL 
    my_drive.axis0.controller.input_torque = torque_input
    Iq_setpoint_current.append(my_drive.axis0.motor.current_control.Iq_setpoint)
    time.sleep(0.01)
    prev_error = error_angle
#    prev_measurement = theta
#    print(my_drive.axis0.encoder.pos_estimate)
    Iq_measured_current.append(my_drive.axis0.motor.current_control.Iq_measured)
    theta_actual.append(my_drive.axis1.encoder.pos_estimate)
    phi_actual.append(my_drive.axis0.encoder.pos_estimate)
if(abs(my_drive.axis0.encoder.pos_estimate > 1)): # Stop the motor if the if one revolution has been completed to prevent the system from becoming unstable
    my_drive.axis0.controller.input_torque = 0    
plt.plot(timerx,pos_plot,label = 'theta_setpoint')
plt.plot(timerx,theta_actual,label = 'theta_actual')
fig = plt.figure()
fig = plt.plot(timerx,Iq_setpoint_current,label='current setpoint')
fig = plt.plot(timerx,Iq_measured_current,label='measured current')
fig1 = plt.figure()
fig1 = plt.plot(timerx,pos_plot1,label='phi_setpoint')
fig1 = plt.plot(timerx,phi_actual,label='phi_actual')
plt.legend()
plt.show()
# with open('Swirling pendulum-trial13.csv', 'w', newline='') as f: # Storing the data in CSV format
#     thewriter = csv.writer(f)
#     thewriter.writerow(['time','pos_plot','theta_actual','Kp','Ki','Kd'])
#     for i in range(0, len(timerx)):
#         thewriter.writerow([timerx[i],pos_plot[i],theta_actual[i],K_p,K_i,K_d])
# I_input = 0
# while(abs(my_drive.axis0.encoder.pos_estimate - abs_zero) > 0.03):
#     print(my_drive.axis0.encoder.pos_estimate)
#     I_input = I_input + 0.01*(my_drive.axis0.encoder.pos_estimate - abs_zero)
#     torque_input = 0.1*(my_drive.axis0.encoder.pos_estimate - abs_zero) + 0.01*(my_drive.axis0.encoder.pos_estimate - abs_zero)
#     my_drive.axis0.controller.input_torque = torque_input
#     if(abs(my_drive.axis0.encoder.pos_estimate - abs_zero)<0.05):
#         break

my_drive.axis0.controller.input_torque = 0 #To stop the motor at the end of the trial
quit()