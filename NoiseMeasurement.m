clear;clc;

h = 1e-3;

%Simulation Duration
Tsim = 30;
%Time Vector 
t = [0:h:Tsim]';
%Input Vector
u = zeros(size(t));
simin = [t,u]; %This goes into simulink
%Start Simulation
sim festotemplate

%Collect output data

%Collect output data: deflection and deflection velocity
data = simout2.Data(3:end); % deflection
data1 = simout3.Data(3:end); % velocity
time = simout2.Time(3:end);   % timestamps


combined = [t u data data1];

writematrix(combined, 'NoiseMeasurement.csv')

x_laser = -(simout1.Data -mean(simout1.Data(1)));
x_actuator = simout.Data - mean(simout.Data(1));
Beam_deflection = x_laser-x_actuator;
subplot(1,2,1)
plot(t,x_laser,t,x_actuator)
legend("Cantilever Position", "Cart Position")
subplot(1,2,2)
plot(t, Beam_deflection)