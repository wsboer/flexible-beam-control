clear;clc;

h = 1e-3;

%Simulation Duration
Tsim = 30;
%Time Vector 
t = [0:h:Tsim]';
%Input Vector
amplitude = 5;
omega = 10;
phase = 0;
u = amplitude*sin(omega*t+phase); %Change this s.t. first 2 seconds have zero input to give time for B switch
%Gain from Model:
K = [-0.3972   -0.8279];

simin = [t,u]; %This goes into simulink
%Start Simulation
sim festotemplate_controller

%Collect output data

x_laser = -(simout1.Data -mean(simout1.Data(1)));
x_actuator = simout.Data - mean(simout.Data(1));
Beam_deflection = x_laser-x_actuator;
subplot(1,2,1)
plot(t,x_laser,t,x_actuator)
legend("Cantilever Position", "Cart Position")
subplot(1,2,2)
plot(t, Beam_deflection)