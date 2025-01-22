close all
clear
clc
%% Swirling Pendulum: Trajectores near Equilibrium Points
m1 = 0.435;
m2 = 0.325;
l1 = 0.07;
l2 = 0.0855;
g=9.81;
Xeq={[pi;0]};    
A=[0 0 1 0;0 0 0 1;-92.5253 -62.9094 0 0;26.9446 75.4913 0 0];
B=[0;0;-270.6055;626.1394];
K=lqr(A,B,5*diag([1,50,100,50]),50);
k=1;
xeq=Xeq{1,k};
% theta_0=xeq(1,:);
% theta_0=xeq(1,:)+0.35074390;
%%
thetaQ=pi;
phiQ=0;
dotthetaQ=0;
dotphiQ=0;
f1=0;f2=0;
% Inertia Matrix
M=[4/3*(m1*l1^2+m2*(3*l1^2+(l2*sin(phiQ))^2)), 2*m2*l1*l2*cos(phiQ);
    2*m2*l1*l2*cos(phiQ), 4/3*(m2*l2^2)];
% Coriolis and centrifugal forces
%CC=[0 8/3*(m2*l2^2)*sin(phiQ)*cos(phiQ)*dotthetaQ-2*m2*l1*l2*sin(phiQ)*dotphiQ;
 %   -4/3*m2*l2^2*cos(phiQ)*sin(phiQ)*dotthetaQ 0];
CC=[4/3*(m2*l2^2)*sin(phiQ)*cos(phiQ)*dotthetaQ 4/3*(m2*l2^2)*sin(phiQ)*cos(phiQ)*dotthetaQ-2*m2*l1*l2*sin(phiQ)*dotphiQ;
    -4/3*m2*l2^2*cos(phiQ)*sin(phiQ)*dotthetaQ 0];
% Potential Terms
G=[(m1+2*m2)*g*l1*sin(thetaQ)+m2*g*l2*sin(phiQ)*cos(thetaQ);
    m2*g*l2*sin(thetaQ)*cos(phiQ)];
BB=[0;1];
%Mx''+Cx'+G=Bu;
% partial(G)/partial(q
partialG_partialq=[(m1+2*m2)*g*l1*cos(thetaQ)-m2*g*l2*sin(thetaQ)*sin(phiQ) m2*g*l2*cos(phiQ)*cos(thetaQ);
    m2*g*l2*cos(phiQ)*cos(thetaQ) -m2*g*l2*sin(thetaQ)*sin(phiQ)];
%% Linear Matrices and Transfer Function

A=[zeros(2,2) eye(2); -inv(M)*partialG_partialq -inv(M)*CC]
B=[zeros(2,1);inv(M)*BB]
C=[1 0 0 0];
D=zeros(size(C*B));
% K=lqr(A,B,1*diag([1,1,1,1]),1000)
% % K=place(A,B,[-0.3 -0.25 -8 -10])
%%
theta_0=xeq(1,:)+-pi+1;
dtheta_0=0;
phi_0=xeq(2,:);
dphi_0=0.01;
sim('SwPenSUSim_SUUS.slx')
figure()
subplot(221)
plot(tout,theta,'LineWidth',1.5)
grid on
ylabel('\theta')
subplot(223)
plot(tout,dottheta,'LineWidth',1.5)
ylabel('{\theta}_{dot}')
xlabel('Time (s)')
grid on
subplot(222)
plot(tout,phi,'LineWidth',1.5)
ylabel('\phi')
grid on
subplot(224)
plot(tout,dotphi,'LineWidth',1.5)
ylabel('{\phi}_{dot}')
grid on
xlabel('Time (s)')