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

print("Starting torque control...")
my_drive.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL

# Torque Limits
torque_input_UL = 3
torque_input_LL  = -3

# Experiment Duration
duration = 24.5

# Trajectory Parameters
w = 3.141 #Frequency of sine wave4
amp = 0.3

# Gains
Kp = 50
Kd = 1.5

# System Parameters
m1 = 0.465
m2 = 0.086
l1 = 0.13
l2 = 0.145
g = 9.81


# Frequent Calculations 
l1_sqr = round(math.pow(l1,2),3) # l1^2 rounded to 3 decimal places
l2_sqr = round(math.pow(l2,2),3) # l2^2 rounded to 3 decimal places
m1l1_sqr = round(m1*l1_sqr,3) # m1*l1^2 product rounded to 3 decimal places
m2l2_sqr = round(m2*l2_sqr,3) # m2*l2^2 product rounded to 3 decimal places
four_by_73 = round(4/3,3) # Round 4/3 to three decimal places
m2l1l2 = round(m2*l1*l2,3) # Round m2*l1*l2 product to 3 decimal places
m2l1_sqr = round(m2*l1_sqr,3) #Round m2*l1^2 product to 3 decimal places
m1plustwom2sum = round((m1+2*m2)*g*l1,3) #Round (m1+2*m2)*g*l1 to 3 decimal places
m2gl2product = round(m2*g*l2,3) #Round m2*g*l2 to 3 decimal places

sigma_a_1 = round(8*m1l1_sqr, 3)
sigma_a_2 = round(24*m2l1_sqr, 3)
sigma_a_3a = round(8*m2l2_sqr, 3)
sigma_a_4a = round(18*m2l1_sqr, 3)
sigma_a_3 = 0
sigma_a_4 = 0
sigma_a = 0
sigma_aa = 0

sigma_b_1 = 0
sigma_b_1a = -1*round(8*m2l2_sqr, 3)
sigma_b_2a = round(6*g*l1*(m1plustwom2sum), 3)
sigma_b_3a = -1*round(12*m2l1l2, 3)
sigma_b_4a = round(6*g*m2*l2, 3)
sigma_b_5a = -1*round(9*g*l1*m2, 3)
sigma_b_6a = round(16*m2l2_sqr, 3)
sigma_b_7a = round(12*m2l1l2, 3)

sigma_c = 0

# Kalman Filter Parameters
#estimate = t3[0]
e_est = 0.1
q = 0.00001
# est = 1 #initial estimate of velocity
P = 0.5
Q = math.exp(-6)
R = 0.8
vel_est = 0.25 #Initial estimate of phi velocity
vel_est1 = 0.01 #Initial estimate of theta velocity


# Arrays for Data Storage
phi_current = round(my_drive.axis0.encoder.pos_estimate*2*math.pi,3) # initial phi angle in radians
phi_previous = phi_current #To store previous value of phi to help in velocity estimation
theta_current = round(my_drive.axis1.encoder.pos_estimate*2*math.pi,3) # initial theta angle in radians
theta_previous = theta_current #To store previous value of theta to help in velocity estimation
phivel_previous = 0
thetavel_previous = 0
theta_actual = []
phi_actual = []
theta_desired = []
timerx = []
theta_vel = []
phi_vel = []
control_ip = []
theta_vel_desired = []
phivel_filtered_output = 0
phivel_mea_previous = 0
phivel_filtered_output_previous = 0
thetavel_filtered_output = 0
thetavel_mea_previous = 0
thetavel_filtered_output_previous = 0
theta_des = 0

# Loop Starting time
t_start = my_drive.system_stats.uptime # start time of trial
my_drive.axis0.encoder.set_linear_count(0)
my_drive.axis1.encoder.set_linear_count(0)
# Control Loop
while(True):

    # Current Time
    time_x = (my_drive.system_stats.uptime - t_start)/1000 # current time in seconds
    sphi = math.sin(phi_current) #Sine of phi in upto 3 decimal places
    cphi = math.cos(phi_current) #Cosine of phi upto 3 decimal places
    s2phi = math.pow(sphi,2) #square of sin(phi) upto 3 decimal places
    c2phi = math.pow(cphi,2) #square of cos(phi) upto 3 decimal places
    stheta = math.sin(phi_current) #Sine of theta upto 3 decimal placesodrv
    ctheta = math.cos(theta_current) #Cosine of theta upto 3 decimal places
    
    # Trajectory Calculation

    # Sine wave
    # phi_setpoint = 0.5*round(math.sin(w*time_x),3)

    # Square wave
    if time_x < 10.5:
        theta_des = amp * round(math.sin(w*time_x),3)
        theta_vel_desired.append(math.cos(w*time_x)*amp*w)
    if (time_x < 15.5 and time_x >= 10.5):
        theta_des = amp 
        theta_vel_desired.append(0)
    if time_x >= 15.5:
        theta_des = -(time_x - 15.5)*(2*amp/9) 
        theta_vel_desired.append(2*amp/9)
        

    theta_setpoint = theta_des*1

    # Controller Calculation
    theta_error = theta_setpoint - theta_current
    u = Kp*theta_error - Kd*thetavel_previous #command for ddotphi to follow

    # Calculation for Controller
    # Sigma A
    sigma_a_3 = round(sigma_a_3a*s2phi, 3)
    sigma_a_4 = round(sigma_a_4a*c2phi, 3)

    sigma_aa = sigma_a_1 + sigma_a_2 + sigma_a_3 - sigma_a_4 # For Sigma B and C
    sigma_a = round(sigma_aa*l2, 3)
    
    # Sigma B
    sigma_b_1 = sigma_aa + round(sigma_b_1a*c2phi, 3)
    sigma_b_2 = round(sigma_b_2a*stheta, 3)
    sigma_b_3 = round(sigma_b_3a*sphi*phivel_previous*phivel_previous, 3)
    sigma_b_4 = round(sigma_b_4a*sphi*ctheta, 3)
    sigma_b_5 = round(sigma_b_5a*stheta*c2phi, 3)
    sigma_b_6 = round(sigma_b_6a*sphi*cphi*phivel_previous*thetavel_previous, 3)
    sigma_b_7 = round(sigma_b_7a*thetavel_previous*thetavel_previous*c2phi*sphi, 3)

    sigma_b = sigma_b_1*u + sigma_b_2 + sigma_b_3 + sigma_b_4 + sigma_b_5 + sigma_b_6 + sigma_b_7

    # Sigma C
    sigma_c = round(sigma_b_1*cphi*9*l1, 3)

    if sigma_c != 0:
        sigma_c_i = round(1/sigma_c, 3)
    else:
        sigma_c_i = 0

    # Control Input 
    torque = -1*round((sigma_a*sigma_b*sigma_c_i), 3)

    print(torque, " and", sigma_c)

    # Saturation limit
    if(torque > torque_input_UL): 
        torque = torque_input_UL
    if(torque < torque_input_LL):
        torque = torque_input_LL 

    my_drive.axis0.controller.input_torque = torque

    t1 = my_drive.system_stats.uptime

    # Position and Velocity Calculation
    phi_current = round(my_drive.axis0.encoder.pos_estimate*2*math.pi,3)
    theta_current = round(my_drive.axis1.encoder.pos_estimate*2*math.pi,3)
    dt = (my_drive.system_stats.uptime - t1)/1000

    if(dt == 0): # If change in time is 0 then the velocities should be same as previous
        thetavel_mea = thetavel_previous
        phivel_mea = phi_previous
        # phivel_mea_previous = phivel_mea
        # thetavel_mea_previous = thetavel_mea
    else:
        thetavel_mea = round(((theta_current-theta_previous)/dt),3) # Insantaneous Theta velocity estimation
        phivel_mea = (phi_current - phi_previous)/dt # Instantaneous Phi Velocity estimation
        # vel_filtered_output = 0.9359*phivel_filtered_output_previous + 0.064*phivel_mea_previous
        # thetavel_filtered_output = 0.9359*thetavel_filtered_output_previous + 0.064*thetavel_mea_previous

    # Saving Position and Calculation
    # print("phi velocity = ",phivel_mea)
    # previous_estimate = vel_est
    # P = P + Q
    # KG = P/(P+R)
    # vel_est = vel_est + KG*(vel_filtered_output - vel_est)
    # P = (1-KG)*P + q*abs(previous_estimate - vel_est)
    phivel_previous = phivel_mea
    # previous_estimate1 = vel_est1
    # P = P + Q
    # KG_theta = P/(P+R)
    # vel_est1 = vel_est1 + KG_theta*(thetavel_mea - vel_est1)
    # P = (1-KG_theta)*P + q*abs(previous_estimate1-vel_est1)
    thetavel_previous = thetavel_mea
    phi_previous = phi_current
    theta_previous = theta_current
    # thetavel_mea_previous = thetavel_filtered_output
    # thetavel_filtered_output_previous = thetavel_filtered_output
    # phivel_mea_previous = phivel_mea
    # phivel_filtered_output_previous = vel_filtered_output
    timerx.append(time_x)
    theta_actual.append(theta_current)
    theta_vel.append(thetavel_previous)
    phi_actual.append(phi_current)
    phi_vel.append(phivel_previous)
    control_ip.append(torque)
    theta_desired.append(theta_setpoint)
    
    if(time_x > duration):
        break

# Deactivating Torque mode
my_drive.axis0.controller.input_torque = 0 #To stop the motor at the end of the trial
my_drive.axis0.requested_state = AXIS_STATE_IDLE #to put the motor in idle state at the end of rotation

# Position Plot
fig = plt.figure()
plt.plot(timerx,theta_desired,label='desired')
plt.plot(timerx,theta_actual,label='actual')
plt.xlabel("Time in seconds")
plt.ylabel("Theta in radians")
plt.legend()

# Velocity Plot
fig1 = plt.figure()
plt.plot(timerx,theta_vel_desired,label='desired')
plt.plot(timerx,theta_vel,label='actual')
plt.xlabel("Time in seconds")
plt.ylabel("Angular velocity in radians per second")
plt.legend()

# Torque Plot
fig2 = plt.figure()
plt.plot(timerx,control_ip,label='control ip')
plt.xlabel("Time in seconds")
plt.ylabel("Torque in Nm")
plt.legend()
plt.show()

# Data Export
# x = str(datetime.datetime.now())
# x = x.replace(":","-")
# y = ('Swirling pendulum IOlinearisation -{}.csv').format(x)
# with open(y, 'w', newline='') as f: # Storing the data in CSV format
#     thewriter = csv.writer(f)
#     thewriter.writerow(['time','theta_desired','thetavel_desired','theta_actual','thetavel_actual','phi_actual', 'phivel','Torque', 'kp', 'kd'])
#     for i in range(0, len(timerx)):
#         thewriter.writerow([timerx[i],theta_desired[i],theta_vel_desired[i],theta_actual[i],theta_vel[i],phi_actual[i],phi_vel[i],control_ip[i],Kp,Kd])


print('done')
quit()
