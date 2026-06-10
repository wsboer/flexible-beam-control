clear;clc;

h = 1e-3;

%Simulation Duration
Tsim = 60;
%Time Vector 
t = [0:h:Tsim]';
%Input Vector
amplitude = 10;
omega = 30;
phase = 0;
u_cycle = amplitude*sin(omega*t+phase); %Change this s.t. first 2 seconds have zero input to give time for B switch
u_step = zeros(size(t)); 
Tpulse = 0.5;
v=amplitude/Tpulse;

u_step(t >= 1 & t < 1 + Tpulse) = v;
%Gain from Model:
Klqr = [0 0];

u = u_step;%Choose which input to use
simin = [t,u]; %This goes into simulink
%Start Simulation
sim festotemplate_controller

%Collect output data

%Collect output data
data = simout.Data; 
data1 = simout1.Data; % signal values
time = simout1.Time;   % timestamps

combined = [time u data data1];

writematrix(combined, 'StepDampingData5.csv')

x_laser = -(simout1.Data -mean(simout1.Data(1)));
x_actuator = simout.Data - mean(simout.Data(1));
Beam_deflection = x_laser-x_actuator;
subplot(1,2,1)
plot(t,x_laser,t,x_actuator)
legend("Cantilever Position", "Cart Position")
subplot(1,2,2)
plot(t, Beam_deflection)