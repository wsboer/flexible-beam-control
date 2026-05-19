clear; clc;

h = 1e-3;

%Create chirp/white noise inputs for resonance experiment:

%Simulation Duration
Tsim = 30;
%Time Vector 
t = [0:h:Tsim]';
%Input Vector
%normalised white noise:
u_white = randn(size(t));
u_white = u_white / max(abs(u_white));
%Chirp signal (more setup-friendly):
w0 = 0.5;%start frequency [rad/s]
w1 = 40 ;%end frequency
u_chirp = chirp(t,w0/(2*pi),Tsim-1,w1/(2*pi));

u = u_chirp; %which input we are using

simin = [t,u]; %This goes into simulink
%Start Simulation
sim festotemplate

%Collect output data
data = simout.Data; 
data1 = simout1.Data% signal values
time = simout1.Time;   % timestamps


combined = [time u data data1];

writematrix(combined, 'ResonanceData.csv')

x_laser = simout1.Data -mean(simout1.Data);
x_actuator = simout.Data + mean(simout.Data);
Beam_deflection = x_laser-x_actuator;
subplot(1,2,1)
plot(t,x_laser,t,x_actuator)
legend("Cantilever Position", "Cart Position")
subplot(1,2,2)
plot(t, Beam_deflection)