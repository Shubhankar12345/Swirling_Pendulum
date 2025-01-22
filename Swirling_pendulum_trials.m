trial1 = csvread('Swirling pendulum-trial1.csv',1,1);
t = 0:0.05:5;
t = t(:,1:100);
t = t' ;
phi_setpoint_trial1 = trial1(:,1);
phi_actual_trial1 = trial1(:,2);
Kp_trial1 = 0.1 ;
Ki_trial1 = 0.1 ;
Kd_trial1 = 0 ;
trial2 = csvread('Swirling pendulum-trial2.csv',1,1);
phi_setpoint_trial2 = trial2(:,1);
phi_actual_trial2 = trial2(:,2);
Kp = trial2(:,3);
Ki = trial2(:,4);
Kp_trial2 = sum(Kp)/length(Kp);
Ki_trial2 = sum(Ki)/length(Ki);
Kd_trial2 = 0 ;
trial3 = csvread('Swirling pendulum-trial3.csv',1,1);
phi_setpoint_trial3 = trial3(:,1);
phi_actual_trial3 = trial3(:,2);
Kp = trial3(:,3);
Ki = trial3(:,4);
Kp_trial3 = sum(Kp)/length(Kp);
Ki_trial3 = sum(Ki)/length(Ki);
Kd_trial3 = 0 ;
trial4 = csvread('Swirling pendulum-trial4.csv',1,1);
phi_setpoint_trial4 = trial4(:,1);
phi_actual_trial4 = trial4(:,2);
Kp = trial4(:,3);
Ki = trial4(:,4);
Kp_trial4 = sum(Kp)/length(Kp);
Ki_trial4 = sum(Ki)/length(Ki);
Kd_trial4 = 0 ;
trial5 = csvread('Swirling pendulum-trial5.csv',1,1);
phi_setpoint_trial5 = trial5(:,1);
phi_actual_trial5 = trial5(:,2);
Kp = trial5(:,3);
Ki = trial5(:,4);
Kp_trial5 = sum(Kp)/length(Kp);
Ki_trial5 = sum(Ki)/length(Ki);
Kd_trial5 = 0 ;
trial6 = csvread('Swirling pendulum-trial6.csv',1,1);
phi_setpoint_trial6 = trial6(:,1);
phi_actual_trial6 = trial6(:,2);
Kp = trial6(:,3);
Ki = trial6(:,4);
Kp_trial6 = sum(Kp)/length(Kp);
Ki_trial6 = sum(Ki)/length(Ki);
Kd_trial6 = 0 ;
trial7 = csvread('Swirling pendulum-trial7.csv',1,1);
phi_setpoint_trial7 = trial7(:,1);
phi_actual_trial7 = trial7(:,2);
Kp = trial7(:,3);
Ki = trial7(:,4);
Kd = trial7(:,5);
Kp_trial7 = sum(Kp)/length(Kp);
Ki_trial7 = sum(Ki)/length(Ki);
Kd_trial7 = sum(Kd)/length(Kd);
trial8 = csvread('Swirling pendulum-trial8.csv',1,1);
phi_setpoint_trial8 = trial8(:,1);
phi_actual_trial8 = trial8(:,2);
Kp = trial8(:,3);
Ki = trial8(:,4);
Kd = trial8(:,5);
Kp_trial8 = sum(Kp)/length(Kp);
Ki_trial8 = sum(Ki)/length(Ki);
Kd_trial8 = sum(Kd)/length(Kd);
trial9 = csvread('Swirling pendulum-trial9.csv',1,1);
phi_setpoint_trial9 = trial9(:,1);
phi_actual_trial9 = trial9(:,2);
Kp = trial9(:,3);
Ki = trial9(:,4);
Kd = trial9(:,5);
Kp_trial9 = sum(Kp)/length(Kp);
Ki_trial9 = sum(Ki)/length(Ki);
Kd_trial9 = sum(Kd)/length(Kd);
trial10 = csvread('Swirling pendulum-trial10.csv',1,1);
phi_setpoint_trial10 = trial10(:,1);
phi_actual_trial10 = trial10(:,2);
Kp = trial10(:,3);
Ki = trial10(:,4);
Kd = trial10(:,5);
Kp_trial10 = sum(Kp)/length(Kp);
Ki_trial10 = sum(Ki)/length(Ki);
Kd_trial10 = sum(Kd)/length(Kd);
trial11 = csvread('Swirling pendulum-trial11.csv',1,1);
phi_setpoint_trial11 = trial11(:,1);
phi_actual_trial11 = trial11(:,2);
Kp = trial11(:,3);
Ki = trial11(:,4);
Kd = trial11(:,5);
Kp_trial11 = sum(Kp)/length(Kp);
Ki_trial11 = sum(Ki)/length(Ki);
Kd_trial11 = sum(Kd)/length(Kd);
trial12 = csvread('Swirling pendulum-trial12.csv',1,1);
phi_setpoint_trial12 = trial12(:,1);
phi_actual_trial12 = trial12(:,2);
Kp = trial12(:,3);
Ki = trial12(:,4);
Kd = trial12(:,5);
Kp_trial12 = sum(Kp)/length(Kp);
Ki_trial12 = sum(Ki)/length(Ki);
Kd_trial12 = sum(Kd)/length(Kd);
trial13 = csvread('Swirling pendulum-trial13.csv',1,1);
phi_setpoint_trial13 = trial13(:,1);
phi_actual_trial13 = trial13(:,2);
Kp = trial13(:,3);
Ki = trial13(:,4);
Kd = trial13(:,5);
Kp_trial13 = sum(Kp)/length(Kp);
Ki_trial13 = sum(Ki)/length(Ki);
Kd_trial13 = sum(Kd)/length(Kd);
subplot(4,4,1);
plot(t,phi_setpoint_trial1,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial1,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial1 Kp=',num2str(Kp_trial1),' Ki=',num2str(Ki_trial1),' Kd=',num2str(Kd_trial1)))
leg1 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg1,'Interpreter','tex');
subplot(4,4,2);
plot(t,phi_setpoint_trial2,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial2,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial2 Kp=',num2str(Kp_trial2),' Ki=',num2str(Ki_trial2),' Kd=',num2str(Kd_trial2)))
leg2 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg2,'Interpreter','tex');
subplot(4,4,3);
plot(t,phi_setpoint_trial3,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial3,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial3 Kp=',num2str(Kp_trial3),' Ki=',num2str(Ki_trial3),' Kd=',num2str(Kd_trial3)))
leg3 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg3,'Interpreter','tex');
subplot(4,4,4);
plot(t,phi_setpoint_trial4,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial4,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial4 Kp=',num2str(Kp_trial4),' Ki=',num2str(Ki_trial4),' Kd=',num2str(Kd_trial4)))
leg4 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg4,'Interpreter','tex');
subplot(4,4,5);
plot(t,phi_setpoint_trial5,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial5,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial5 Kp=',num2str(Kp_trial5),' Ki=',num2str(Ki_trial5),' Kd=',num2str(Kd_trial5)))
leg5 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg5,'Interpreter','tex');
subplot(4,4,6);
plot(t,phi_setpoint_trial6,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial6,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial6 Kp=',num2str(Kp_trial6),' Ki=',num2str(Ki_trial6),' Kd=',num2str(Kd_trial6)))
leg6 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg6,'Interpreter','tex');
subplot(4,4,7);
plot(t,phi_setpoint_trial7,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial7,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial7 Kp=',num2str(Kp_trial7),' Ki=',num2str(Ki_trial7),' Kd=',num2str(Kd_trial7)))
leg7 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg7,'Interpreter','tex');
subplot(4,4,8);
plot(t,phi_setpoint_trial8,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial8,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial8 Kp=',num2str(Kp_trial8),' Ki=',num2str(Ki_trial8),' Kd=',num2str(Kd_trial8)))
leg8 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg8,'Interpreter','tex');
subplot(4,4,9);
plot(t,phi_setpoint_trial9,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial9,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial9 Kp=',num2str(Kp_trial9),' Ki=',num2str(Ki_trial9),' Kd=',num2str(Kd_trial9)))
leg9 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg9,'Interpreter','tex');
subplot(4,4,9);
plot(t,phi_setpoint_trial9,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial9,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial9 Kp=',num2str(Kp_trial9),' Ki=',num2str(Ki_trial9),' Kd=',num2str(Kd_trial9)))
leg9 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg9,'Interpreter','tex');
subplot(4,4,9);
plot(t,phi_setpoint_trial9,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial9,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial9 Kp=',num2str(Kp_trial9),' Ki=',num2str(Ki_trial9),' Kd=',num2str(Kd_trial9)))
leg9 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg9,'Interpreter','tex');
subplot(4,4,10);
plot(t,phi_setpoint_trial10,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial10,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial10 Kp=',num2str(Kp_trial10),' Ki=',num2str(Ki_trial10),' Kd=',num2str(Kd_trial10)))
leg10 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg10,'Interpreter','tex');
subplot(4,4,11);
plot(t,phi_setpoint_trial11,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial11,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial11 Kp=',num2str(Kp_trial11),' Ki=',num2str(Ki_trial11),' Kd=',num2str(Kd_trial11)))
leg11 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg11,'Interpreter','tex');
subplot(4,4,12);
plot(t,phi_setpoint_trial12,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial2,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial12 Kp=',num2str(Kp_trial12),' Ki=',num2str(Ki_trial12),' Kd=',num2str(Kd_trial12)))
leg12 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg12,'Interpreter','tex');
subplot(4,4,13);
plot(t,phi_setpoint_trial13,'r-','linewidth',1);
hold on
plot(t,phi_actual_trial13,'b--','linewidth',1);
xlabel('time in seconds');
ylabel('phi in no.of turns');
title(strcat('setpoint tracking trial13 Kp=',num2str(Kp_trial13),' Ki=',num2str(Ki_trial13),' Kd=',num2str(Kd_trial13)))
leg13 = legend('\phi_{setpoint}','\phi_{actual}');
set(leg13,'Interpreter','tex');