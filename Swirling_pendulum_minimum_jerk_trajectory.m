trial1 = csvread('Swirling pendulum-minimum jerk trajectory-6.csv',1,1);
t = 0:0.01:5;
t = t' ;
phi_setpoint_trial1_minimum_jerk_trajectory = trial1(:,1);
phi_actual_trial1_minimum_jerk_trajectory = trial1(:,2);
Kp_trial1_minimum_jerk_trajectory = trial1(:,3) ;
Ki_trial1_minimum_jerk_trajectory = trial1(:,4) ;
Kd_trial1_minimum_jerk_trajectory = trial1(:,5) ;
Tau_trial1_minimum_jerk_trajectory = trial1(:,7) ;
Kp_trial1 = sum(Kp_trial1_minimum_jerk_trajectory)/length(Kp_trial1_minimum_jerk_trajectory) ;
Ki_trial1 = sum(Ki_trial1_minimum_jerk_trajectory)/length(Ki_trial1_minimum_jerk_trajectory) ;
Kd_trial1 = sum(Kd_trial1_minimum_jerk_trajectory)/length(Kd_trial1_minimum_jerk_trajectory) ;
Tau_trial1 = sum(Tau_trial1_minimum_jerk_trajectory)/length(Tau_trial1_minimum_jerk_trajectory) ; % Derivative filter constant
trial2 = csvread('Swirling pendulum-minimum jerk trajectory-7.csv',1,1);
phi_setpoint_trial2_minimum_jerk_trajectory = trial2(:,1);
phi_actual_trial2_minimum_jerk_trajectory = trial2(:,2);
Kp_trial2_minimum_jerk_trajectory = trial2(:,3) ;
Ki_trial2_minimum_jerk_trajectory = trial2(:,4) ;
Kd_trial2_minimum_jerk_trajectory = trial2(:,5) ;
Tau_trial2_minimum_jerk_trajectory = trial2(:,7) ;
Kp_trial2 = sum(Kp_trial2_minimum_jerk_trajectory)/length(Kp_trial2_minimum_jerk_trajectory) ;
Ki_trial2 = sum(Ki_trial2_minimum_jerk_trajectory)/length(Ki_trial2_minimum_jerk_trajectory) ;
Kd_trial2 = sum(Kd_trial2_minimum_jerk_trajectory)/length(Kd_trial2_minimum_jerk_trajectory) ;
Tau_trial2 = sum(Tau_trial2_minimum_jerk_trajectory)/length(Tau_trial2_minimum_jerk_trajectory) ; % Derivative filter constant
trial3 = csvread('Swirling pendulum-minimum jerk trajectory-8.csv',1,1);
phi_setpoint_trial3_minimum_jerk_trajectory = trial3(:,1);
phi_actual_trial3_minimum_jerk_trajectory = trial3(:,2);
Kp_trial3_minimum_jerk_trajectory = trial3(:,3) ;
Ki_trial3_minimum_jerk_trajectory = trial3(:,4) ;
Kd_trial3_minimum_jerk_trajectory = trial3(:,5) ;
Tau_trial3_minimum_jerk_trajectory = trial3(:,7) ;
Kp_trial3 = sum(Kp_trial3_minimum_jerk_trajectory)/length(Kp_trial3_minimum_jerk_trajectory) ;
Ki_trial3 = sum(Ki_trial3_minimum_jerk_trajectory)/length(Ki_trial3_minimum_jerk_trajectory) ;
Kd_trial3 = sum(Kd_trial3_minimum_jerk_trajectory)/length(Kd_trial3_minimum_jerk_trajectory) ;
Tau_trial3 = sum(Tau_trial3_minimum_jerk_trajectory)/length(Tau_trial3_minimum_jerk_trajectory) ; % Derivative filter constant
trial4 = csvread('Swirling pendulum-minimum jerk trajectory-9.csv',1,1);
phi_setpoint_trial4_minimum_jerk_trajectory = trial4(:,1);
phi_actual_trial4_minimum_jerk_trajectory = trial4(:,2);
Kp_trial4_minimum_jerk_trajectory = trial4(:,3) ;
Ki_trial4_minimum_jerk_trajectory = trial4(:,4) ;
Kd_trial4_minimum_jerk_trajectory = trial4(:,5) ;
Tau_trial4_minimum_jerk_trajectory = trial4(:,7) ;
Kp_trial4 = sum(Kp_trial4_minimum_jerk_trajectory)/length(Kp_trial4_minimum_jerk_trajectory) ;
Ki_trial4 = sum(Ki_trial4_minimum_jerk_trajectory)/length(Ki_trial4_minimum_jerk_trajectory) ;
Kd_trial4 = sum(Kd_trial4_minimum_jerk_trajectory)/length(Kd_trial4_minimum_jerk_trajectory) ;
Tau_trial4 = sum(Tau_trial4_minimum_jerk_trajectory)/length(Tau_trial4_minimum_jerk_trajectory) ; % Derivative filter constant
trial5 = csvread('Swirling pendulum-minimum jerk trajectory-10.csv',1,1);
phi_setpoint_trial5_minimum_jerk_trajectory = trial5(:,1);
phi_actual_trial5_minimum_jerk_trajectory = trial5(:,2);
Kp_trial5_minimum_jerk_trajectory = trial5(:,3) ;
Ki_trial5_minimum_jerk_trajectory = trial5(:,4) ;
Kd_trial5_minimum_jerk_trajectory = trial5(:,5) ;
Tau_trial5_minimum_jerk_trajectory = trial5(:,7) ;
Kp_trial5 = sum(Kp_trial5_minimum_jerk_trajectory)/length(Kp_trial5_minimum_jerk_trajectory) ;
Ki_trial5 = sum(Ki_trial5_minimum_jerk_trajectory)/length(Ki_trial5_minimum_jerk_trajectory) ;
Kd_trial5 = sum(Kd_trial5_minimum_jerk_trajectory)/length(Kd_trial5_minimum_jerk_trajectory) ;
Tau_trial5 = sum(Tau_trial5_minimum_jerk_trajectory)/length(Tau_trial5_minimum_jerk_trajectory) ; % Derivative filter constant
trial6 = csvread('Swirling pendulum-minimum jerk trajectory-11.csv',1,1);
phi_setpoint_trial6_minimum_jerk_trajectory = trial6(:,1);
phi_actual_trial6_minimum_jerk_trajectory = trial6(:,2);
Kp_trial6_minimum_jerk_trajectory = trial6(:,3) ;
Ki_trial6_minimum_jerk_trajectory = trial6(:,4) ;
Kd_trial6_minimum_jerk_trajectory = trial6(:,5) ;
Tau_trial6_minimum_jerk_trajectory = trial6(:,7) ;
Kp_trial6 = sum(Kp_trial6_minimum_jerk_trajectory)/length(Kp_trial6_minimum_jerk_trajectory) ;
Ki_trial6 = sum(Ki_trial6_minimum_jerk_trajectory)/length(Ki_trial6_minimum_jerk_trajectory) ;
Kd_trial6 = sum(Kd_trial6_minimum_jerk_trajectory)/length(Kd_trial6_minimum_jerk_trajectory) ;
Tau_trial6 = sum(Tau_trial6_minimum_jerk_trajectory)/length(Tau_trial6_minimum_jerk_trajectory) ; % Derivative filter constant
trial7 = csvread('Swirling pendulum-minimum jerk trajectory-12.csv',1,1);
phi_setpoint_trial7_minimum_jerk_trajectory = trial7(:,1);
phi_actual_trial7_minimum_jerk_trajectory = trial7(:,2);
Kp_trial7_minimum_jerk_trajectory = trial7(:,3) ;
Ki_trial7_minimum_jerk_trajectory = trial7(:,4) ;
Kd_trial7_minimum_jerk_trajectory = trial7(:,5) ;
Tau_trial7_minimum_jerk_trajectory = trial7(:,7) ;
Kp_trial7 = sum(Kp_trial7_minimum_jerk_trajectory)/length(Kp_trial7_minimum_jerk_trajectory) ;
Ki_trial7 = sum(Ki_trial7_minimum_jerk_trajectory)/length(Ki_trial7_minimum_jerk_trajectory) ;
Kd_trial7 = sum(Kd_trial7_minimum_jerk_trajectory)/length(Kd_trial7_minimum_jerk_trajectory) ;
Tau_trial7 = sum(Tau_trial7_minimum_jerk_trajectory)/length(Tau_trial7_minimum_jerk_trajectory) ; % Derivative filter constant
trial8 = csvread('Swirling pendulum-minimum jerk trajectory-13.csv',1,1);
phi_setpoint_trial8_minimum_jerk_trajectory = trial8(:,1);
phi_actual_trial8_minimum_jerk_trajectory = trial8(:,2);
Kp_trial8_minimum_jerk_trajectory = trial8(:,3) ;
Ki_trial8_minimum_jerk_trajectory = trial8(:,4) ;
Kd_trial8_minimum_jerk_trajectory = trial8(:,5) ;
Tau_trial8_minimum_jerk_trajectory = trial8(:,7) ;
Kp_trial8 = sum(Kp_trial8_minimum_jerk_trajectory)/length(Kp_trial8_minimum_jerk_trajectory) ;
Ki_trial8 = sum(Ki_trial8_minimum_jerk_trajectory)/length(Ki_trial8_minimum_jerk_trajectory) ;
Kd_trial8 = sum(Kd_trial8_minimum_jerk_trajectory)/length(Kd_trial8_minimum_jerk_trajectory) ;
Tau_trial8 = sum(Tau_trial8_minimum_jerk_trajectory)/length(Tau_trial8_minimum_jerk_trajectory) ; % Derivative filter constant
trial9 = csvread('Swirling pendulum-minimum jerk trajectory-14.csv',1,1);
phi_setpoint_trial9_minimum_jerk_trajectory = trial9(:,1);
phi_actual_trial9_minimum_jerk_trajectory = trial9(:,2);
Kp_trial9_minimum_jerk_trajectory = trial9(:,3) ;
Ki_trial9_minimum_jerk_trajectory = trial9(:,4) ;
Kd_trial9_minimum_jerk_trajectory = trial9(:,5) ;
Tau_trial9_minimum_jerk_trajectory = trial9(:,7) ;
Kp_trial9 = sum(Kp_trial9_minimum_jerk_trajectory)/length(Kp_trial9_minimum_jerk_trajectory) ;
Ki_trial9 = sum(Ki_trial9_minimum_jerk_trajectory)/length(Ki_trial9_minimum_jerk_trajectory) ;
Kd_trial9 = sum(Kd_trial9_minimum_jerk_trajectory)/length(Kd_trial9_minimum_jerk_trajectory) ;
Tau_trial9 = sum(Tau_trial9_minimum_jerk_trajectory)/length(Tau_trial9_minimum_jerk_trajectory) ; % Derivative filter constant
trial10 = csvread('Swirling pendulum-minimum jerk trajectory-15.csv',1,1);
phi_setpoint_trial10_minimum_jerk_trajectory = trial10(:,1);
phi_actual_trial10_minimum_jerk_trajectory = trial10(:,2);
Kp_trial10_minimum_jerk_trajectory = trial10(:,3) ;
Ki_trial10_minimum_jerk_trajectory = trial10(:,4) ;
Kd_trial10_minimum_jerk_trajectory = trial10(:,5) ;
Tau_trial10_minimum_jerk_trajectory = trial10(:,7) ;
Kp_trial10 = sum(Kp_trial10_minimum_jerk_trajectory)/length(Kp_trial10_minimum_jerk_trajectory) ;
Ki_trial10 = sum(Ki_trial10_minimum_jerk_trajectory)/length(Ki_trial10_minimum_jerk_trajectory) ;
Kd_trial10 = sum(Kd_trial10_minimum_jerk_trajectory)/length(Kd_trial10_minimum_jerk_trajectory) ;
Tau_trial10 = sum(Tau_trial10_minimum_jerk_trajectory)/length(Tau_trial10_minimum_jerk_trajectory) ; % Derivative filter constant
trial11 = csvread('Swirling pendulum-minimum jerk trajectory-16.csv',1,1);
phi_setpoint_trial11_minimum_jerk_trajectory = trial11(:,1);
phi_actual_trial11_minimum_jerk_trajectory = trial11(:,2);
Kp_trial11_minimum_jerk_trajectory = trial11(:,3) ;
Ki_trial11_minimum_jerk_trajectory = trial11(:,4) ;
Kd_trial11_minimum_jerk_trajectory = trial11(:,5) ;
Tau_trial11_minimum_jerk_trajectory = trial11(:,7) ;
Kp_trial11 = sum(Kp_trial11_minimum_jerk_trajectory)/length(Kp_trial11_minimum_jerk_trajectory) ;
Ki_trial11 = sum(Ki_trial11_minimum_jerk_trajectory)/length(Ki_trial11_minimum_jerk_trajectory) ;
Kd_trial11 = sum(Kd_trial11_minimum_jerk_trajectory)/length(Kd_trial11_minimum_jerk_trajectory) ;
Tau_trial11 = sum(Tau_trial11_minimum_jerk_trajectory)/length(Tau_trial11_minimum_jerk_trajectory) ; % Derivative filter constant
trial12 = csvread('Swirling pendulum-minimum jerk trajectory-17.csv',1,1);
phi_setpoint_trial12_minimum_jerk_trajectory = trial12(:,1);
phi_actual_trial12_minimum_jerk_trajectory = trial12(:,2);
Kp_trial12_minimum_jerk_trajectory = trial12(:,3) ;
Ki_trial12_minimum_jerk_trajectory = trial12(:,4) ;
Kd_trial12_minimum_jerk_trajectory = trial12(:,5) ;
Tau_trial12_minimum_jerk_trajectory = trial12(:,7) ;
Kp_trial12 = sum(Kp_trial12_minimum_jerk_trajectory)/length(Kp_trial12_minimum_jerk_trajectory) ;
Ki_trial12 = sum(Ki_trial12_minimum_jerk_trajectory)/length(Ki_trial12_minimum_jerk_trajectory) ;
Kd_trial12 = sum(Kd_trial12_minimum_jerk_trajectory)/length(Kd_trial2_minimum_jerk_trajectory) ;
Tau_trial12 = sum(Tau_trial12_minimum_jerk_trajectory)/length(Tau_trial12_minimum_jerk_trajectory) ; % Derivative filter constant
subplot(4,3,1)
plot(t,phi_setpoint_trial1_minimum_jerk_trajectory,'r-','linewidth',1)
hold on
plot(t,phi_actual_trial1_minimum_jerk_trajectory,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in radians');
title(strcat('Minimum jerk trajectory tracking Kp=',num2str(Kp_trial1),' Ki=',num2str(Ki_trial1),' Kd=',num2str(Kd_trial1),' Tau=',num2str(Tau_trial1)))
leg1 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg1,'Interpreter','tex');
subplot(4,3,2)
plot(t,phi_setpoint_trial2_minimum_jerk_trajectory,'r-','linewidth',1)
hold on
plot(t,phi_actual_trial2_minimum_jerk_trajectory,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in radians');
title(strcat('Minimum jerk trajectory tracking-2 Kp=',num2str(Kp_trial2),' Ki=',num2str(Ki_trial2),' Kd=',num2str(Kd_trial2),' Tau=',num2str(Tau_trial2)))
leg2 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg2,'Interpreter','tex');
subplot(4,3,3)
plot(t,phi_setpoint_trial3_minimum_jerk_trajectory,'r-','linewidth',1)
hold on
plot(t,phi_actual_trial3_minimum_jerk_trajectory,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in radians');
title(strcat('Minimum jerk trajectory tracking-3 Kp=',num2str(Kp_trial3),' Ki=',num2str(Ki_trial3),' Kd=',num2str(Kd_trial3),' Tau=',num2str(Tau_trial3)))
leg3 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg3,'Interpreter','tex');
subplot(4,3,4)
plot(t,phi_setpoint_trial4_minimum_jerk_trajectory,'r-','linewidth',1)
hold on
plot(t,phi_actual_trial4_minimum_jerk_trajectory,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in radians');
title(strcat('Minimum jerk trajectory tracking-4 Kp=',num2str(Kp_trial4),' Ki=',num2str(Ki_trial4),' Kd=',num2str(Kd_trial4),' Tau=',num2str(Tau_trial4)))
leg4 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg4,'Interpreter','tex');
subplot(4,3,5)
plot(t,phi_setpoint_trial5_minimum_jerk_trajectory,'r-','linewidth',1)
hold on
plot(t,phi_actual_trial5_minimum_jerk_trajectory,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in radians');
title(strcat('Minimum jerk trajectory tracking-5 Kp=',num2str(Kp_trial5),' Ki=',num2str(Ki_trial5),' Kd=',num2str(Kd_trial5),' Tau=',num2str(Tau_trial5)))
leg5 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg5,'Interpreter','tex');
subplot(4,3,6)
plot(t,phi_setpoint_trial6_minimum_jerk_trajectory,'r-','linewidth',1)
hold on
plot(t,phi_actual_trial6_minimum_jerk_trajectory,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in radians');
title(strcat('Minimum jerk trajectory tracking-6 Kp=',num2str(Kp_trial6),' Ki=',num2str(Ki_trial6),' Kd=',num2str(Kd_trial6),' Tau=',num2str(Tau_trial6)))
leg6 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg6,'Interpreter','tex');
subplot(4,3,7)
plot(t,phi_setpoint_trial7_minimum_jerk_trajectory,'r-','linewidth',1)
hold on
plot(t,phi_actual_trial7_minimum_jerk_trajectory,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in radians');
title(strcat('Minimum jerk trajectory tracking-7 Kp=',num2str(Kp_trial7),' Ki=',num2str(Ki_trial7),' Kd=',num2str(Kd_trial7),' Tau=',num2str(Tau_trial7)))
leg7 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg7,'Interpreter','tex');
subplot(4,3,8)
plot(t,phi_setpoint_trial8_minimum_jerk_trajectory,'r-','linewidth',1)
hold on
plot(t,phi_actual_trial8_minimum_jerk_trajectory,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in radians');
title(strcat('Minimum jerk trajectory tracking-8 Kp=',num2str(Kp_trial8),' Ki=',num2str(Ki_trial8),' Kd=',num2str(Kd_trial8),' Tau=',num2str(Tau_trial8)))
leg8 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg8,'Interpreter','tex');
subplot(4,3,9)
plot(t,phi_setpoint_trial9_minimum_jerk_trajectory,'r-','linewidth',1)
hold on
plot(t,phi_actual_trial9_minimum_jerk_trajectory,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in radians');
title(strcat('Minimum jerk trajectory tracking-9 Kp=',num2str(Kp_trial9),' Ki=',num2str(Ki_trial9),' Kd=',num2str(Kd_trial9),' Tau=',num2str(Tau_trial9)))
leg9 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg9,'Interpreter','tex');
subplot(4,3,10)
plot(t,phi_setpoint_trial10_minimum_jerk_trajectory,'r-','linewidth',1)
hold on
plot(t,phi_actual_trial10_minimum_jerk_trajectory,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in radians');
title(strcat('Minimum jerk trajectory tracking-10 Kp=',num2str(Kp_trial10),' Ki=',num2str(Ki_trial10),' Kd=',num2str(Kd_trial10),' Tau=',num2str(Tau_trial10)))
leg10 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg10,'Interpreter','tex');
subplot(4,3,11)
plot(t,phi_setpoint_trial11_minimum_jerk_trajectory,'r-','linewidth',1)
hold on
plot(t,phi_actual_trial11_minimum_jerk_trajectory,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in radians');
title(strcat('Minimum jerk trajectory tracking-11 Kp=',num2str(Kp_trial11),' Ki=',num2str(Ki_trial11),' Kd=',num2str(Kd_trial11),' Tau=',num2str(Tau_trial11)))
leg11 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg11,'Interpreter','tex');
subplot(4,3,12)
plot(t,phi_setpoint_trial12_minimum_jerk_trajectory,'r-','linewidth',1)
hold on
plot(t,phi_actual_trial12_minimum_jerk_trajectory,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in radians');
title(strcat('Minimum jerk trajectory tracking-12 Kp=',num2str(Kp_trial12),' Ki=',num2str(Ki_trial12),' Kd=',num2str(Kd_trial12),' Tau=',num2str(Tau_trial12)))
leg12 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg12,'Interpreter','tex');
