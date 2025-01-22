clc;
clear all;
trial1 = csvread('Swirling pendulum minimum jerk trajectory tracking -2022-03-05 14-36-21.558375.csv',1,0);
time1 = trial1(:,1) ;
phi_desired = trial1(:,2) ;
phi_actual1 = trial1(:,3) ;
theta_actual1 = trial1(:,4) ;
torque1 = trial1(:,5) ;
Kp_1 = trial1(:,7) ;
Ki_1 = trial1(:,8) ;
Kd_1 = trial1(:,9) ;
Kp_1 = sum(Kp_1)/length(Kp_1) ;
Ki_1 = sum(Ki_1)/length(Ki_1) ;
Kd_1 = sum(Kd_1)/length(Kd_1) ;
trial2 = csvread('Swirling pendulum minimum jerk trajectory tracking -2022-03-05 14-39-00.350560.csv',1,0);
time2 = trial2(:,1) ;
phi_actual2 = trial2(:,3) ;
theta_actual2 = trial2(:,4) ;
torque2 = trial2(:,5) ;
Kp_2 = trial2(:,7) ;
Ki_2 = trial2(:,8) ;
Kd_2 = trial2(:,9) ;
Kp_2 = sum(Kp_2)/length(Kp_2) ;
Ki_2 = sum(Ki_2)/length(Ki_2) ;
Kd_2 = sum(Kd_2)/length(Kd_2) ;
trial3 = csvread('Swirling pendulum minimum jerk trajectory tracking -2022-03-05 14-40-02.739048.csv',1,0);
time3 = trial3(:,1) ;
phi_actual3 = trial3(:,3) ;
theta_actual3 = trial3(:,4) ;
torque3 = trial3(:,5) ;
Kp_3 = trial3(:,7) ;
Ki_3 = trial3(:,8) ;
Kd_3 = trial3(:,9) ;
Kp_3 = sum(Kp_3)/length(Kp_3) ;
Ki_3 = sum(Ki_3)/length(Ki_3) ;
Kd_3 = sum(Kd_3)/length(Kd_3) ;
trial4 = csvread('Swirling pendulum minimum jerk trajectory tracking -2022-03-05 14-40-53.896608.csv',1,0);
time4 = trial4(:,1) ;
phi_actual4 = trial4(:,3) ;
theta_actual4 = trial4(:,4) ;
torque4 = trial4(:,5) ;
Kp_4 = trial4(:,7) ;
Ki_4 = trial4(:,8) ;
Kd_4 = trial4(:,9) ;
Kp_4 = sum(Kp_4)/length(Kp_4) ;
Ki_4 = sum(Ki_4)/length(Ki_4) ;
Kd_4 = sum(Kd_4)/length(Kd_4) ;
trial5 = csvread('Swirling pendulum minimum jerk trajectory tracking for unstable equilibrium -2022-03-05 14-59-23.541178.csv',1,0);
time5 = trial5(:,1) ;
phi_desired2 = trial5(:,2) ;
phi_actual5 = trial5(:,3) ;
theta_actual5 = trial5(:,4) ;
torque5 = trial5(:,5) ;
Kp_5 = trial5(:,7) ;
Ki_5 = trial5(:,8) ;
Kd_5 = trial5(:,9) ;
Kp_5 = sum(Kp_5)/length(Kp_5) ;
Ki_5 = sum(Ki_5)/length(Ki_5) ;
Kd_5 = sum(Kd_5)/length(Kd_5) ;
trial6 = csvread('Swirling pendulum minimum jerk trajectory tracking for unstable equilibrium -2022-03-05 15-01-09.574558.csv',1,0);
time6 = trial6(:,1) ;
phi_actual6 = trial6(:,3) ;
theta_actual6 = trial6(:,4) ;
torque6 = trial6(:,5) ;
Kp_6 = trial6(:,7) ;
Ki_6 = trial6(:,8) ;
Kd_6 = trial6(:,9) ;
Kp_6 = sum(Kp_5)/length(Kp_6) ;
Ki_6 = sum(Ki_5)/length(Ki_6) ;
Kd_6 = sum(Kd_6)/length(Kd_6) ;
trial7 = csvread('Swirling pendulum minimum jerk trajectory tracking for unstable equilibrium -2022-03-05 16-02-54.512537.csv',1,0);
time7 = trial7(:,1) ;
phi_actual7 = trial7(:,3) ;
theta_actual7 = trial7(:,4) ;
torque7 = trial7(:,5) ;
Kp_7 = trial7(:,7) ;
Ki_7 = trial7(:,8) ;
Kd_7 = trial7(:,9) ;
Kp_7 = sum(Kp_7)/length(Kp_7) ;
Ki_7 = sum(Ki_7)/length(Ki_7) ;
Kd_7 = sum(Kd_7)/length(Kd_7) ;
trial8 = csvread('Swirling pendulum minimum jerk trajectory tracking for unstable equilibrium -2022-03-05 16-33-38.408395.csv',1,0);
time8 = trial8(:,1) ;
phi_actual8 = trial8(:,3) ;
theta_actual8 = trial8(:,4) ;
torque8 = trial8(:,5) ;
Kp_8 = trial8(:,7) ;
Ki_8 = trial8(:,8) ;
Kd_8 = trial8(:,9) ;
Kp_8 = sum(Kp_8)/length(Kp_8) ;
Ki_8 = sum(Ki_8)/length(Ki_8) ;
Kd_8 = sum(Kd_8)/length(Kd_8) ;
subplot(2,2,1) ;
plot(time1,phi_desired,'r-','linewidth',1);
hold on
plot(time1,phi_actual1,'k--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('Minimum jerk trajectory for equilibrium transfer Kp=',num2str(Kp_1),' Ki=',num2str(Ki_1),' Kd=',num2str(Kd_1)))
leg4 = legend('\phi_{desired}','\phi_{actual}');
set(leg4,'Interpreter','tex');
subplot(2,2,2) ;
plot(time1,phi_desired,'r-','linewidth',1);
hold on
plot(time2,phi_actual2,'k--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('Minimum jerk trajectory for equilibrium transfer Kp=',num2str(Kp_2),' Ki=',num2str(Ki_2),' Kd=',num2str(Kd_2)))
leg4 = legend('\phi_{desired}','\phi_{actual}');
set(leg4,'Interpreter','tex');
subplot(2,2,3) ;
plot(time1,phi_desired,'r-','linewidth',1);
hold on
plot(time3,phi_actual3,'k--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('Minimum jerk trajectory for equilibrium transfer Kp=',num2str(Kp_3),' Ki=',num2str(Ki_3),' Kd=',num2str(Kd_3)))
leg4 = legend('\phi_{desired}','\phi_{actual}');
set(leg4,'Interpreter','tex');
subplot(2,2,4) ;
plot(time1,phi_desired,'r-','linewidth',1);
hold on
plot(time4,phi_actual4,'k--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('Minimum jerk trajectory for equilibrium transfer Kp=',num2str(Kp_4),' Ki=',num2str(Ki_4),' Kd=',num2str(Kd_4)))
leg4 = legend('\phi_{desired}','\phi_{actual}');
set(leg4,'Interpreter','tex');
% figure ;
% subplot(2,2,1) ;
% plot(time5,phi_desired2,'r-','linewidth',1);
% hold on
% plot(time5,phi_actual5,'k--','linewidth',1);
% xlabel('time in seconds');
% ylabel('phi in no.of turns');
% title(strcat('Minimum jerk trajectory for unstable equilibrium transfer Kp=',num2str(Kp_5),' Ki=',num2str(Ki_5),' Kd=',num2str(Kd_5)))
% leg4 = legend('\phi_{desired}','\phi_{actual}');
% set(leg4,'Interpreter','tex');
% subplot(2,2,2) ;
% plot(time5,phi_desired2,'r-','linewidth',1);
% hold on
% plot(time6,phi_actual6,'k--','linewidth',1);
% xlabel('time in seconds');
% ylabel('phi in no.of turns');
% title(strcat('Minimum jerk trajectory for unstable equilibrium transfer Kp=',num2str(Kp_6),' Ki=',num2str(Ki_6),' Kd=',num2str(Kd_6)))
% leg4 = legend('\phi_{desired}','\phi_{actual}');
% set(leg4,'Interpreter','tex');
% subplot(2,2,3) ;
% plot(time5,phi_desired2,'r-','linewidth',1);
% hold on
% plot(time7,phi_actual7,'k--','linewidth',1);
% xlabel('time in seconds');
% ylabel('phi in no.of turns');
% title(strcat('Minimum jerk trajectory for unstable equilibrium transfer Kp=',num2str(Kp_7),' Ki=',num2str(Ki_7),' Kd=',num2str(Kd_7)))
% leg4 = legend('\phi_{desired}','\phi_{actual}');
% set(leg4,'Interpreter','tex');
% subplot(2,2,4) ;
% plot(time5,phi_desired2,'r-','linewidth',1);
% hold on
% plot(time8,phi_actual8,'k--','linewidth',1);
% xlabel('time in seconds');
% ylabel('phi in no.of turns');
% title(strcat('Minimum jerk trajectory for unstable equilibrium transfer Kp=',num2str(Kp_8),' Ki=',num2str(Ki_8),' Kd=',num2str(Kd_8)))
% leg4 = legend('\phi_{desired}','\phi_{actual}');
% set(leg4,'Interpreter','tex');