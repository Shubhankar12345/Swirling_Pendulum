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


# Torque Limits
torque_input_UL = 6
torque_input_LL  = -6

# Arrays to save trajectory
theta_d = [] 
dottheta_d = []
ddottheta_d = []
timerx = [] 

# Trajectory parameters
w = 3.141 #Frequency of sine wave4
amp = 1.5
duration = 40 

# Gains
# Kp = 500
# Kd = 1.5

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
four_by_3 = round(4/3,3) # Round 4/3 to three decimal places
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

#Kalman Filter parameters
P1 = 0.0001
P2 = P1
R = 0.009 
q1 = 0.005
q2 = q1

# # Trajectory generation 
# while(True):
#     # Current Time
#     time_x = (my_drive.system_stats.uptime - t_start)/1000 # current time in seconds
#     tdiff = (time_x*1000) - t_prev
#     if(time_x > duration):
#         break
#     if(tdiff >= 24):
#         if(time_x < 10):
#             theta_setpoint = round(amp*math.sin(w*time_x),3)
#             dottheta_setpoint = round(amp*w*math.cos(w*time_x),3)
#             ddottheta_setpoint = round(-amp*math.pow(w,2.0)*math.sin(w*time_x),3)
#             theta_d.append(theta_setpoint)
#             dottheta_d.append(dottheta_setpoint)
#             ddottheta_d.append(ddottheta_setpoint)
#         elif(time_x >= 10 and time_x < 20):  
#             t1 = time_x - 10
#             theta_setpoint = 0.03*t1
#             dottheta_setpoint = 0.03
#             ddottheta_setpoint = 0.0
#             theta_d.append(theta_setpoint)
#             dottheta_d.append(dottheta_setpoint)
#             ddottheta_d.append(ddottheta_setpoint)        
#         elif(time_x >= 20 and time_x < 30):
#             theta_setpoint = 0.3
#             dottheta_setpoint = 0.0
#             ddottheta_setpoint = 0.0
#             theta_d.append(theta_setpoint)
#             dottheta_d.append(dottheta_setpoint)
#             ddottheta_d.append(ddottheta_setpoint)   
#         else:
#             t2 = time_x - 30
#             theta_setpoint = -0.03*t2 + 0.3
#             dottheta_setpoint = -0.03
#             ddottheta_setpoint = 0.0
#             theta_d.append(theta_setpoint)
#             dottheta_d.append(dottheta_setpoint)
#             ddottheta_d.append(ddottheta_setpoint)  
#         timerx.append(time_x)
#         t_prev = time_x*1000
# print('trajectory generation done')
# Arrays for Data Storage
phi_current = round(my_drive.axis0.encoder.pos_estimate*2*math.pi,3) # initial phi angle in radians
theta_current = round(my_drive.axis1.encoder.pos_estimate*2*math.pi,3) # initial theta angle in radians
theta_actual = []
phi_actual = []
theta_vel = []
phi_vel = []
err = []
control_ip = []
sigma_c1111 = []
sigma_c_prev = 0
phivel_est = 0
thetavel_est = (amp*w)/(2*math.pi)
my_drive.axis0.encoder.set_linear_count(0)
my_drive.axis1.encoder.set_linear_count(0)
print("Starting closed loop control...")
my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

print("Starting torque control...")
my_drive.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
t_start = my_drive.system_stats.uptime # start time of trial
t_prev = 0 
thetavel_mea= 0
# Control Loop
while(True):
    # Current Time
    time_x = (my_drive.system_stats.uptime - t_start)/1000 # current time in seconds
    if(time_x > duration):
        break
    if(time_x < 10):
        theta_setpoint = round((amp*math.sin(w*time_x))/(2*math.pi),3)
        dottheta_setpoint = round((amp*w*math.cos(w*time_x))/(2*math.pi),3)*0
        ddottheta_setpoint = round((-amp*math.pow(w,2.0)*math.sin(w*time_x))/(2*math.pi),3)*0
        u = ddottheta_setpoint
        Kp = 500
        Kd = 1.5
    elif(time_x >= 10 and time_x < 20):  
        t1 = time_x - 10
        theta_setpoint = (0.03*t1)/(2*math.pi)
        dottheta_setpoint = 0.03/(2*math.pi)*0
        ddottheta_setpoint = 0.0
        u = ddottheta_setpoint    
        Kp = 5
        Kd = 1
    elif(time_x >= 20 and time_x < 30):
        theta_setpoint = 0.3/(2*math.pi)
        dottheta_setpoint = 0.0
        ddottheta_setpoint = 0.0
        u = ddottheta_setpoint
        Kp = 50
        Kd = 10 
    else:
        t2 = time_x - 30
        theta_setpoint = (-0.03*t2 + 0.3)/(2*math.pi)
        dottheta_setpoint = -0.03*0
        ddottheta_setpoint = 0.0
        u = ddottheta_setpoint
        Kp = 50
        Kd = 10

    sphi = math.sin(phi_current) #Sine of phi in upto 3 decimal places
    cphi = math.cos(phi_current) #Cosine of phi upto 3 decimal places
    s2phi = math.pow(sphi,2) #square of sin(phi) upto 3 decimal places
    c2phi = math.pow(cphi,2) #square of cos(phi) upto 3 decimal places
    stheta = math.sin(phi_current) #Sine of theta upto 3 decimal placesodrv
    ctheta = math.cos(theta_current) #Cosine of theta upto 3 decimal places

    # Controller Calculation
    theta_error = theta_setpoint - theta_current
    dottheta_error = -thetavel_mea
    u = u + Kp*theta_error + Kd*dottheta_error #command for ddottheta to follow

    # Calculation for Controller
    # Sigma A
    sigma_a_3 = round(sigma_a_3a*s2phi, 3)
    sigma_a_4 = round(sigma_a_4a*c2phi, 3)

    sigma_aa = sigma_a_1 + sigma_a_2 + sigma_a_3 - sigma_a_4 # For Sigma B and C
    sigma_a = round(sigma_aa*l2, 3)
    
    # Sigma B
    sigma_b_1 = sigma_aa + round(sigma_b_1a*c2phi, 3)
    sigma_b_2 = round(sigma_b_2a*stheta, 3)
    sigma_b_3 = round(sigma_b_3a*sphi*phivel_est*phivel_est, 3)
    sigma_b_4 = round(sigma_b_4a*sphi*ctheta, 3)
    sigma_b_5 = round(sigma_b_5a*stheta*c2phi, 3)
    sigma_b_6 = round(sigma_b_6a*sphi*cphi*phivel_est*thetavel_est, 3)
    sigma_b_7 = round(sigma_b_7a*thetavel_est*thetavel_est*c2phi*sphi, 3)

    sigma_b = sigma_b_1*u + sigma_b_2 + sigma_b_3 + sigma_b_4 + sigma_b_5 + sigma_b_6 + sigma_b_7

    # Sigma C
    sigma_c = round(sigma_b_1*9*l1, 3)

    # sigma_c_i = round(1/sigma_c, 3)
    if sigma_c != 0:
        sigma_c_i = round(1/sigma_c, 3)
    else:
        sigma_c_i = 0

    # Control Input 
    torque = -1*round((sigma_a*sigma_b*sigma_c_i), 3)

    # Saturation limit
    if(torque > torque_input_UL): 
        torque = torque_input_UL
    if(torque < torque_input_LL):
        torque = torque_input_LL 

    my_drive.axis0.controller.input_torque = torque
    # Position and Velocity Calculation
    phi_current = round(my_drive.axis0.encoder.pos_estimate,3)
    theta_current = round(my_drive.axis1.encoder.pos_estimate,3)
    phivel_mea = round(my_drive.axis0.encoder.vel_estimate,3)
    thetavel_mea = round(my_drive.axis1.encoder.vel_estimate,3)
    
    # # Kalman smoothing of velocity
    # vel_est1 = phivel_est
    # KG_phi = P1/(P1+R)
    # phivel_est = phivel_est + KG_phi*(phivel_mea - phivel_est)
    # P1 = (1-KG_phi)*P1 + q1*abs(vel_est1-phivel_est)    

    # vel_est2 = thetavel_est
    # KG_theta = P2/(P2+R)
    # thetavel_est = thetavel_est + KG_theta*(thetavel_mea - thetavel_est)
    # P2 = (1-KG_theta)*P2 + q2*abs(vel_est2-thetavel_est)      

    t_prev = time_x*1000
    # Storing the current values
    theta_d.append(theta_setpoint)
    dottheta_d.append(dottheta_setpoint)
    ddottheta_d.append(ddottheta_setpoint)  
    theta_actual.append(theta_current)
    theta_vel.append(thetavel_mea)
    phi_actual.append(phi_current)
    phi_vel.append(phivel_mea)
    control_ip.append(torque)
    timerx.append(time_x)
    sigma_c1111.append(sigma_c_i)    
    err.append(theta_error)
# Deactivating Torque mode
my_drive.axis0.controller.input_torque = 0 #To stop the motor at the end of the trial
my_drive.axis0.requested_state = AXIS_STATE_IDLE #to put the motor in idle state at the end of rotation
print("Odrive in Idle mode")

# Position Plot
fig1 = plt.figure()
plt.plot(timerx,theta_d,label='desired')
plt.plot(timerx,theta_actual,label='actual')
plt.xlabel("Time in seconds")
plt.ylabel("Theta in radians")
plt.legend()

# Velocity Plot
fig2 = plt.figure()
plt.plot(timerx,dottheta_d,label='desired')
plt.plot(timerx,theta_vel,label='actual')
plt.xlabel("Time in seconds")
plt.ylabel("Angular velocity in radians per second")
plt.legend()

# Torque Plot
fig3 = plt.figure()
plt.plot(timerx,control_ip,label='control ip')
plt.xlabel("Time in seconds")
plt.ylabel("Torque in Nm")
plt.legend()

#error plot
fig4 = plt.figure()
plt.plot(timerx,err,label='sigma_c')
plt.xlabel("Time in seconds")
plt.ylabel("joint error")
plt.legend()
plt.show()

# Data Export
# x = str(datetime.datetime.now())
# x = x.replace(":","-")
# y = ('Swirling pendulum IOlinearisation -{}.csv').format(x)
# with open(y, 'w', newline='') as f: # Storing the data in CSV format
#     thewriter = csv.writer(f)
#     thewriter.writerow(['time','theta_desired','theta_actual','thetavel_actual','phi_actual', 'phivel','Torque', 'kp', 'kd'])
#     for i in range(0, len(timerx)):
#         thewriter.writerow([timerx[i],theta_desired[i],theta_actual[i],theta_vel[i],phi_actual[i],phi_vel[i],control_ip[i],Kp,Kd])


print('done')
quit()