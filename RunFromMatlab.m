clear;clc;

h = 1e-3;

%Simulation Duration
Tsim = 30;
%Time Vector 
t = [0:h:Tsim]';
%Input Vector
amplitude = 2;
omega = 20;%2*pi*5.5;
phase = 0;
u_cycle = amplitude*sin(omega*t+phase); %Change this s.t. first 2 seconds have zero input to give time for B switch
u_step = zeros(size(t)); 
Tpulse = 0.05;
v=1/Tpulse;

u_step(t >= 1 & t < 1 + Tpulse) = v;

u = u_cycle;%Choose which input to use
simin = [t,u]; %This goes into simulink
%LQG Parameters:

%Klqr =[-73.3318    0.1663];
%Klqr =[-248.9683    1.9379];
%Klqr =[-129.2267    0.5200];%

%Klqr =[-153.1577    0.7315];%25 25
Klqr =[-102.8469    0.3287]; %15 15
%Klqr = [0    0.000];
A = [[0 1]
    [-(2*pi*5.5)^2 -2*2*pi*5.5*0.7499]];
B = [-0.07528015464
    0];
C = [[0.5 0]
    [0 1]];
D = [0
    0];
L = [0.2304   -4.2865
   -2.1432   61.9543];

Ak = A-L*C;
Bk = [B L];
Ck = eye(2);
Dk = 0*[B L];

%Start Simulation
sim festotemplate

%Collect output data

figure
x_laser = -(simout1.Data -mean(simout1.Data(1)));
x_actuator = simout.Data - mean(simout.Data(1));
Beam_deflection = x_laser-x_actuator;
subplot(1,2,1)
plot(t,x_laser,t,x_actuator)
legend("Cantilever Position", "Cart Position")
subplot(1,2,2)
title("Beam Deflection")
plot(t, Beam_deflection)


deflection = simout2.Data(3:end);
deflection_velocity = simout3.Data(3:end);

figure
plot(t,deflection,t,deflection_velocity)
ylim([-0.15,0.15])
legend("Beam Deflection", "Beam Deflection Velocity")
grid on

amplitude_deflection = max(abs(deflection))

combined = [t u deflection deflection_velocity];

%writematrix(combined, 'ParameterEvaluation.csv')

