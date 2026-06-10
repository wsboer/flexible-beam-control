clear;clc;

h = 1e-3;

%Simulation Duration
Tsim = 15;
%Time Vector 
t = [0:h:Tsim]';
%Input Vector
amplitude = 10;
omega = 11*pi;%2*pi*5.5;
phase = 0;
u_cycle = amplitude*sin(omega*t+phase); %Change this s.t. first 2 seconds have zero input to give time for B switch
u_step = zeros(size(t)); 
Tpulse = 0.05;
v=30/Tpulse;

u_step(t >= 1 & t < 1 + Tpulse*0.75) = v;

u_step(t >= 2 & t < 2 + Tpulse*0.75) = -v;

u_step(t >= 4 & t < 4 + Tpulse) = 1.5*v;

u_step(t >= 8 & t < 8 + Tpulse) = -v;

u_step(t >= 11 & t < 11 + Tpulse) = v;




u = 0*u_cycle ;%Choose which input to use 
simin = [t,u]; %This goes into simulink
%LQG Parameters:

%Klqr =[-73.3318    0.1663];
%Klqr =[-248.9683    1.9379];
%Klqr =[-129.2267    0.5200];%

%Klqr =[-153.1577    0.7315];%25 25 t_settle = 11
%Klqr = [-248.6818    1.9413]; %25 50 0.1
Klqr = [-333.6516    3.4961]; % 40 80 0.1
%Klqr =[-102.8469    0.3287]; %15 15*
%Klqr =[-174.9101    0.9595]; % 15 30

%Klqr =[-24.5920    0.0186]; % 15 30 R = 1*
%Klqr =[-46.7152    0.0678]; % 15 30 R = 0.5
%Klqr = [-57.0188    0.1013];% 15 30 R = 0.4
%Klqr = [-39.5795    0.0486];% 15 30 R = 0.6 
%Klqr = [-34.3413    0.0365]; % 15 30 R = 0.7 t=27.95
%Klqr = [-30.3308    0.0284]; % 15 30 R = 0.8 
%Klqr =[-80.8856    0.2056]; % 15 90 R = 0.8 
%Klqr = [0    0.000]; %t = 24.83
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


deflection = simout2.Data(end-size(t)+1:end);
deflection_velocity = simout3.Data(end-size(t)+1:end);

figure
plot(t,deflection,t,deflection_velocity)
ylim([-0.15,0.15])
legend("Beam Deflection", "Beam Deflection Velocity")
grid on

amplitude_deflection = max(abs(deflection))

combined = [t u deflection deflection_velocity];

damp_data = abs(deflection(1538:end));

t_damp = t(1538:end);
%Method 1
threshold = 0.10*max(damp_data);
t_settling = 0.0;
for i=1:(length(damp_data)-180)
    if max(damp_data(i:i+180)) <= threshold
        t_settling = t_damp(i)
        break
    end
end

t_settling

writematrix(combined, 'NoControllerFreeOscillation.csv')

