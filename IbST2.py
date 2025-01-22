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
torque_input_UL = 0.6
torque_input_LL  = -0.6
    
# Experiment Duration
duration = 40

# Trajectory Parameters
w_theta = 6.282 #Frequency of sine wave, theta
amp_theta = 0.3
w_phi = 6.28 #Frequency of sine wave, phi
amp_phi = 0.4

# # Gains
# Kp = 500
# Kd = 1.5

# System Parameters
m1 = 0.465
m2 = 0.086
l1 = 0.13
l2 = 0.145
g = 9.81

# Discrete A, B, C Matrix

A = np.array([[0.9831, -0.003197, 0.02486, -2.667*math.exp(-5)], [0.006948, 1.004, 5.795*math.exp(-5), 0.02504], [-1.35, -0.2552, 0.9831, -0.003197], [0.5547, 0.3446, 0.006948, 1.004]])
B = np.array([[-0.03522], [0.1771], [-2.817], [14.17]])

C = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
D = np.array([[0], [0], [0], [0]])

# C bar Matrix

Cd1 = np.matmul(C[0], A)
Cd2 = np.matmul(C[1], A)
Cd3 = C[2]
Cd4 = C[3]

Cd = np.array([Cd1, Cd2, Cd3, Cd4])

# Multiplication

CB = np.matmul(Cd, B)
CBT = CB.transpose()
CdA = np.matmul(Cd, A)

# W Matrices

W1 = np.array([[90, 0, 0, 0], [0, 0.08, 0, 0], [0, 0, 0.001, 0], [0, 0, 0, 0.08]])
W1T = W1.transpose()
W2 = np.array([[0.0001, 0, 0, 0], [0, 35, 0, 0], [0, 0, 0.0001, 0], [0, 0, 0, 0.1]])
W2T = W2.transpose()
W3 = np.array([[10, 0, 0, 0], [0, 2, 0, 0], [0, 0, 0.0001, 0], [0, 0, 0, 0.0001]])
W3T = W3.transpose()
W4 = np.array([[0.1, 0, 0, 0], [0, 0.1, 0, 0], [0, 0, 20, 0], [0, 0, 0, 0.1]])
W4T = W4.transpose()
W5 = np.array([[0.0001, 0, 0, 0], [0, 0.0001, 0, 0], [0, 0, 0.0001, 0], [0, 0, 0, 50]])
W5T = W5.transpose()
W = W1

# Calculations for Control Torque

u1 = np.matmul(np.linalg.pinv(np.matmul(CBT, np.matmul(W1, CB))), np.matmul(CBT, W1T))
u2 = np.matmul(np.linalg.pinv(np.matmul(CBT, np.matmul(W2, CB))), np.matmul(CBT, W2T))
u3 = np.matmul(np.linalg.pinv(np.matmul(CBT, np.matmul(W3, CB))), np.matmul(CBT, W3T))
u4 = np.matmul(np.linalg.pinv(np.matmul(CBT, np.matmul(W4, CB))), np.matmul(CBT, W4T))
u5 = np.matmul(np.linalg.pinv(np.matmul(CBT, np.matmul(W5, CB))), np.matmul(CBT, W5T))

# Arrays for Data Storage
phi_current = round(my_drive.axis0.encoder.pos_estimate*2*math.pi,3) # initial phi angle in radians
phi_previous = phi_current #To store previous value of phi to help in velocity estimation
theta_current = round(my_drive.axis1.encoder.pos_estimate*2*math.pi,3) # initial theta angle in radians
theta_previous = theta_current #To store previous value of theta to help in velocity estimation

timerx = []
theta_actual = []
phi_actual = []
theta_vel = []
phi_vel = []
control_ip = []

theta_desired = []
theta_vel_desired = []
phi_vel_desired = []
phi_desired = []

phivel_previous = 0
thetavel_previous = 0
phivel_mea_previous = 0
thetavel_mea_previous = 0

theta_des = 0
theta_des_k = 0
theta_vel_des = 0
phi_vel_des = 0
phi_des = 0
phi_des_k = 0

# Loop Starting time
t_prev = 0
t_start = my_drive.system_stats.uptime # start time of trial
x = np.array([[0], [0], [0], [0]])
count = 0
target_sampling_time = 0.025
RT_conversion_factor = round(1/target_sampling_time,3)
# Control Loop
while(True):

    # Current Time
    time_x = (my_drive.system_stats.uptime - t_start)/1000 # current time in seconds
    tdiff = (time_x*1000) - t_prev

    if(time_x > duration):
        break

    if tdiff >= 24:    
        # Trajectory Calculation

        # Theta, Thetadot, Phi, Phidot Desired

        if count < 400:
            theta_des = amp_theta * round(math.sin(w_theta*((count+1)/RT_conversion_factor)),3)
            theta_des_k = amp_theta * round(math.sin(w_theta*((count)/RT_conversion_factor)),3)
            # theta_des = round(amp_theta*0.25*(count + 1)/RT_conversion_factor, 3)
            # theta_des_k = round(amp_theta*0.25*(count)/RT_conversion_factor, 3)
            theta_vel_des = math.sin(w_theta*((count+1)/RT_conversion_factor))*0.1
            theta_vel_des_k = math.sin(w_theta*((count)/RT_conversion_factor))*0.1

            phi_des = (amp_phi) * round(math.sin(w_phi*((count+1)/RT_conversion_factor)),3)
            phi_des_k = (amp_phi) * round(math.sin(w_phi*((count)/RT_conversion_factor)),3)
            phi_vel_des = round(math.cos(w_phi*((count+1)/RT_conversion_factor))*amp_phi*w_phi, 3)
            phi_vel_des_k = round(math.cos(w_phi*((count)/RT_conversion_factor))*amp_phi*w_phi, 3)
            W = W1
            ua = u1

        if (count < 800 and count >= 400):
            theta_des = (amp_theta) * round(math.sin(w_theta*((count+1)/RT_conversion_factor)),3)
            theta_des_k = amp_theta * round(math.sin(w_theta*((count)/RT_conversion_factor)),3)
            # theta_des = 2.5*amp_theta - round(amp_theta*0.25*(count + 1)/RT_conversion_factor, 3)
            # theta_des_k = 2.5*amp_theta - round(amp_theta*0.25*(count)/RT_conversion_factor, 3)
            theta_vel_des = math.cos(w_theta*((count)/RT_conversion_factor))*(amp_theta)*w_theta
            theta_vel_des_k = math.sin(w_theta*((count)/RT_conversion_factor))*0.1

            phi_des = amp_phi * round(math.sin(w_phi*((count+1)/RT_conversion_factor)),3)
            phi_des_k = amp_phi * round(math.sin(w_phi*((count)/RT_conversion_factor)),3)
            phi_vel_des = round(math.cos(w_phi*((count+1)/RT_conversion_factor))*amp_phi*w_phi, 3)
            phi_vel_des_k = round(math.cos(w_phi*((count)/RT_conversion_factor))*amp_phi*w_phi, 3)

            W = W2
            ua = u2
        
        if (count < 1200 and count >= 800):
            theta_des = (amp_theta) * round(math.sin(w_theta*((count+1)/RT_conversion_factor)),3)
            theta_des_k = amp_theta * round(math.sin(w_theta*((count)/RT_conversion_factor)),3)
            theta_vel_des = math.cos(w_theta*((count)/RT_conversion_factor))*(amp_theta)*w_theta
            theta_vel_des_k = math.sin(w_theta*((count)/RT_conversion_factor))*0.1

            phi_des = amp_phi * round(math.sin(w_phi*((count+1)/RT_conversion_factor)),3)
            phi_des_k = amp_phi * round(math.sin(w_phi*((count)/RT_conversion_factor)),3)
            phi_vel_des = round(math.cos(w_phi*((count+1)/RT_conversion_factor))*amp_phi*w_phi, 3)
            phi_vel_des_k = round(math.cos(w_phi*((count)/RT_conversion_factor))*amp_phi*w_phi, 3)

            W = W5
            ua = u5
        if (count < 1600 and count >= 1200):
            theta_des = (amp_theta) * round(math.sin(w_theta*((count+1)/RT_conversion_factor)),3)
            theta_des_k = amp_theta * round(math.sin(w_theta*((count)/RT_conversion_factor)),3)
            theta_vel_des = math.cos(w_theta*((count+1)/RT_conversion_factor))*(amp_theta)*w_theta
            theta_vel_des_k = math.sin(w_theta*((count)/RT_conversion_factor))*0.1

            phi_des = amp_phi * round(math.sin(w_phi*((count+1)/RT_conversion_factor)),3)
            phi_des_k = amp_phi * round(math.sin(w_phi*((count)/RT_conversion_factor)),3)
            phi_vel_des = round(math.cos(w_phi*((count+1)/RT_conversion_factor))*amp_phi*w_phi, 3)
            phi_vel_des_k = round(math.cos(w_phi*((count)/RT_conversion_factor))*amp_phi*w_phi, 3)

            W = W4
            ua = u4

        yref = np.array([[theta_des], [phi_des], [theta_vel_des], [phi_vel_des]])

                # Position and Velocity Calculation
        phi_current = round(my_drive.axis0.encoder.pos_estimate*2*math.pi,3)
        theta_current = round(my_drive.axis1.encoder.pos_estimate*2*math.pi,3)
        # dt = (my_drive.system_stats.uptime - t1)/1000

        # if(dt == 0): # If change in time is 0 then the velocities should be same as previous
        #     thetavel_mea = thetavel_previous
        #     phivel_mea = phi_previous
        # else:
        #     thetavel_mea = round(((theta_current-theta_previous)/dt),3) # Insantaneous Theta velocity estimation
        #     phivel_mea = (phi_current - phi_previous)/dt # Instantaneous Phi Velocity estimation
        phi_vel_mea = my_drive.axis0.encoder.vel_estimate*2*math.pi
        theta_vel_mea = my_drive.axis1.encoder.vel_estimate*2*math.pi

        x = np.array([[theta_current], [phi_current], [theta_vel_mea], [phi_vel_mea]], dtype='f')

        # Control Input 
        torque = np.matmul(ua, (yref - np.matmul(CdA, x)))
        torque = torque[0][0]

        # print(tdiff, 'and', count)
        # print(count, 'and', tdiff)

        # Saturation limit
        if(torque > torque_input_UL): 
            torque = torque_input_UL
        if(torque < torque_input_LL):
            torque = torque_input_LL 

        my_drive.axis0.controller.input_torque = torque

        t1 = my_drive.system_stats.uptime



        # phivel_previous = phivel_mea
        # thetavel_previous = thetavel_mea
        # phi_previous = phi_current
        # theta_previous = theta_current

        timerx.append(time_x)

        theta_actual.append(theta_current)
        theta_vel.append(theta_vel_mea)

        phi_actual.append(phi_current)
        phi_vel.append(phi_vel_mea)
        control_ip.append(torque)

        theta_desired.append(theta_des_k)
        phi_desired.append(phi_des_k)
        theta_vel_desired.append(theta_vel_des_k)
        phi_vel_desired.append(phi_vel_des_k)
        t_prev = time_x*1000

        count = count + 1
print(count)
# Deactivating Torque mode
my_drive.axis0.controller.input_torque = 0 #To stop the motor at the end of the trial
my_drive.axis0.requested_state = AXIS_STATE_IDLE #to put the motor in idle state at the end of rotation

print('idle')
# Position Plot
fig = plt.figure()
plt.plot(timerx,theta_desired,label='desired')
plt.plot(timerx,theta_actual,label='actual')
plt.xlabel("Time in seconds")
plt.ylabel("Theta in radians")
plt.legend()
# plt.show()
print('plotdone')

fig1 = plt.figure()
plt.plot(timerx,phi_desired,label='desired')
plt.plot(timerx,phi_actual,label='actual')
plt.xlabel("Time in seconds")
plt.ylabel("Phi in radians")
plt.legend()
# plt.show()
print('plotdone')

# Velocity Plot
fig2 = plt.figure()
plt.plot(timerx,phi_vel_desired,label='desired')
plt.plot(timerx,phi_vel,label='actual')
plt.xlabel("Time in seconds")
plt.ylabel("Angular velocity of phi in radians per second")
plt.legend()

fig3 = plt.figure()
plt.plot(timerx,theta_vel_desired,label='desired')
plt.plot(timerx,theta_vel,label='actual')
plt.xlabel("Time in seconds")
plt.ylabel("Angular velocity of theta in radians per second")
plt.legend()

# Torque Plot
fig3 = plt.figure()
plt.plot(timerx,control_ip,label='control ip')
plt.xlabel("Time in seconds")
plt.ylabel("Torque in Nm")
plt.legend()
plt.show()


# Data Export
# x = str(datetime.datetime.now())
# x = x.replace(":","-")
# y = ('Swirling pendulum Selective Tracking -{}.csv').format(x)
# with open(y, 'w', newline='') as f: # Storing the data in CSV format
#     thewriter = csv.writer(f)
#     thewriter.writerow(['time','theta_desired','theta_actual', 'thetavel_desired','thetavel_actual','phi_desired', 'phi_actual', 'phivel_desired','phivel','Torque'])
#     for i in range(0, len(timerx)):
#         thewriter.writerow([timerx[i],theta_desired[i],theta_actual[i],theta_vel_desired[i],theta_vel[i],phi_desired[i], phi_actual[i], phi_vel_desired[i], phi_vel[i],control_ip[i]])


print('done')
quit()