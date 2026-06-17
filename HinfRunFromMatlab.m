clear;clc;

h = 1e-3;

%Simulation Duration
Tsim = 20;
%Time Vector 
t = [0:h:Tsim]';
%Input Vector
amplitude = 1;
omega = 20;%2*pi*5.5;
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

u = 0*2*square(10*t) ;%Choose which input to use 
simin = [t,u]; %This goes into simulink

%New 3 state model
wn = 2*pi*5.53; %rad/s
ksi = 0.749; %-
alpha = 0.07528015464;
tau = 0.008; %actuator dynamics included
G = 39;
m2m1 = 3;

Anew = [[0 1 0]
    [-wn^2 -2*ksi*wn 1/tau*(1+m2m1)]
    [0 0 -1/tau]];

Bnew = [[0]
    [(G - (1+m2m1)/tau)]
    [(1/tau)]];

Cnew = [[1/40.33*0.9 0 0]
    [0 1/28.134*1.1 0]
    [0 0 0.026]];

Dnew = [0;0;0];

%H-Infinity Controller Dynamics

sys = ss(Anew,Bnew,Cnew,Dnew);

%Adding integrator to plant due to actuator bias
s = tf('s');
sys_aug = [sys; tf(1/s)*sys(1,:)];
%Weighting functions:
%Sensitivity Weight: W_S, we'd like disturbance rejection at higher
%unmodelled freqs (low-pass filter)
W_S = makeweight(1.1,[wn,0.5],0.1);

%Complementary Sensitivity Weight: W_T, robustness to high-frequency
%uncertainty
W_T = makeweight(0.01,[500,1],5);

%Control Effort Weight: W_U, we'd like minimal/less control effort for same
%stabilising result, so less actuator usage at high frequencies, less
%abrupt
W_U = makeweight(0.001,[100,0.01],0.4);

P = augw(sys_aug, W_S, W_U, W_T);

nmeas = 3;
ncont = 1;
[K, CL, gamma] = hinfsyn(P, nmeas, ncont);

gamma

[AH BH CH DH] = ssdata(K)

figure
sigma(CL(1:3,:))
title('W\_S * S')
grid on

figure
sigma(CL(4,:))
title('W\_U * KS')
grid on

figure
sigma(CL(5:7,:))
title('W\_T * T')
grid on

%Start Simulation
sim festotemplate_controller.slx

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