clear;clc;

h = 1e-3;

%Simulation Duration
Tsim = 10;
%Time Vector 
t = [0:h:Tsim]';

%TRACKING TASK INPUT - Square wave tracking (position)

freq_ref = 5; %rad/s
amp_ref = 4;

r_pos = amp_ref*square(freq_ref*t);

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

u = r_pos;%Choose which input to use 
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

L = [0.0000   -0.0002    0.0000
   -0.0001    0.0053    0.0000
    0.0000    0.0000    0.0000];

Akg = Anew-L*Cnew;
Bkg = [Bnew L];
Ckg = eye(3);
Dkg = 0*[Bnew L];

%Tuning LQR gains with Q and R
%Klqr =[-420.8860  -19.6206   32.6317]; %40 80 40, 0.1
%Klqr =[-131.8105   -6.1589    9.6644]; %40 80 40, 1
%Klqr = [-131.8105   -6.1589 0];
%Klqr = [-424.5442  -20.9101   26.9066]; %40 80 20, 0.1
%Klqr =[-148.0020   -6.8846   10.9516]; %40 10 5, 0.1
%Klqr =[-210.0914   -9.7652   15.8781]; %80 20 10, 0.1
%Klqr =[-102.2262   -4.4279    9.5164]; %100 5 5, 0.1
%Klqr = [-144.8414   -6.2975   13.8064]; %150 10 10, 0.1
Klqr =[-4.7204e+02  -2.4792e+01   2.4350e+01]; %100 100 10, 0.1
%Start Simulation
sim festotemplate.slx

%Collect output data

% figure
x_laser = -(simout1.Data -mean(simout1.Data(1)));
x_actuator = simout.Data - mean(simout.Data(1));
Beam_deflection = x_laser-x_actuator;
% subplot(1,2,1)
% plot(t,x_laser,t,x_actuator)
% legend("Cantilever Position", "Cart Position")
% subplot(1,2,2)
% title("Beam Deflection")
% plot(t, Beam_deflection)


deflection = simout2.Data(end-size(t)+1:end);
deflection_velocity = simout3.Data(end-size(t)+1:end);

figure
subplot(1,2,1)
plot(t,u, 'g-',LineWidth=0.8); hold on
plot(t,x_actuator,'r-',LineWidth=0.8); hold off
grid on
subplot(1,2,2)
plot(t,deflection, 'b-', LineWidth=0.8); hold on
plot(t,deflection_velocity, 'r-', LineWidth=0.8); hold off
ylim([-0.15,0.15])
legend("Beam Deflection", "Beam Deflection Velocity")
grid on





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%OLD CODE - for 2 state system%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% A = [[0 1]
%     [-(2*pi*5.5)^2 -2*2*pi*5.5*0.7499]];
% B = [-0.07528015464
%     0];
% C = [[0.5 0]
%     [0 1]];
% D = [0
%     0];
% L = [0.2304   -4.2865
%    -2.1432   61.9543];

%LQG Parameters:

% %Klqr =[-73.3318    0.1663];
% %Klqr =[-248.9683    1.9379];
% %Klqr =[-129.2267    0.5200];%
% 
% %Klqr =[-153.1577    0.7315];%25 25 t_settle = 11
% %Klqr = [-248.6818    1.9413]; %25 50 0.1
% %Klqr = [-333.6516    3.4961]; % 40 80 0.1
% %Klqr = [-380.0294    4.5362]; %50 100 0.1
% %Klqr =[-102.8469    0.3287]; %15 15*
% %Klqr =[-174.9101    0.9595]; % 15 30
% Klqr =[-380.4670    4.5309];% 100 100 0.1
% 
% %Klqr =[-24.5920    0.0186]; % 15 30 R = 1*
% %Klqr =[-46.7152    0.0678]; % 15 30 R = 0.5
% %Klqr = [-57.0188    0.1013];% 15 30 R = 0.4
% %Klqr = [-39.5795    0.0486];% 15 30 R = 0.6 
% %Klqr = [-34.3413    0.0365]; % 15 30 R = 0.7 t=27.95
% %Klqr = [-30.3308    0.0284]; % 15 30 R = 0.8 
% %Klqr =[-80.8856    0.2056]; % 15 90 R = 0.8 
% %Klqr = [0    0.000]; %t = 24.83
% % 
% amplitude_deflection = max(abs(deflection))
% 
% combined = [t u deflection deflection_velocity];
% 
% damp_data = abs(deflection(1538:end));
% 
% t_damp = t(1538:end);
% %Method 1
% threshold = 0.10*max(damp_data);
% t_settling = 0.0;
% for i=1:(length(damp_data)-180)
%     if max(damp_data(i:i+180)) <= threshold
%         t_settling = t_damp(i)
%         break
%     end
% end
% 
% t_settling

%writematrix(combined, 'NoControllerFreeOscillation.csv')

