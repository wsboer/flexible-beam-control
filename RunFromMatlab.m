clear;clc;

h = 1e-3;

%Simulation Duration
Tsim = 10;
%Time Vector 
t = [0:h:Tsim]';
%Input Vector
amplitude = 1;
omega = 20;
phase = 0;
u_cycle = amplitude*sin(omega*t+phase); %Change this s.t. first 2 seconds have zero input to give time for B switch
u_step = zeros(size(t)); 
Tpulse = 0.05;
v=1/Tpulse;

u_step(t >= 1 & t < 1 + Tpulse) = v;
%Gain from Model:
K = [-0.3972  -0.8279];

u = u_cycle;%Choose which input to use
simin = [t,u]; %This goes into simulink
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
legend("Beam Deflection", "Beam Deflection Velocity")
grid on