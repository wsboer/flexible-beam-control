clear; clc;
format short e

h = 1e-3;

%% System Model
wn   = 2*pi*5.53;
ksi  = 0.749;
tau  = 0.008;
G    = 39;
m2m1 = 3;

Anew = [0,        1,                   0;
       -wn^2, -2*ksi*wn, (1/tau)*(1+m2m1);
        0,        0,              -1/tau];

Bnew = [0; G-(1+m2m1)/tau; 1/tau];

Cnew = [1/40.33*0.9,        0,     0;
         0,       1/28.134*1.1,    0;
         0,               0,   0.026];

Dnew = [0; 0; 0];

%H-Infinity Controller Dynamics

sys = ss(Anew,Bnew,Cnew,Dnew);

%Adding integrator to plant due to actuator bias
s = tf('s');
sys_aug = [sys; tf(1/s)*sys(1,:)];
%Weighting functions:
%Sensitivity Weight: W_S, we'd like disturbance rejection at higher
%unmodelled freqs (low-pass filter)
W_S = makeweight(2,[wn,0.5],0.1);

%Complementary Sensitivity Weight: W_T, robustness to high-frequency
%uncertainty
W_T = makeweight(0.01,[500,1],5);

%Control Effort Weight: W_U, we'd like minimal/less control effort for same
%stabilising result, so less actuator usage at high frequencies, less
%abrupt
W_U = makeweight(0.0001,[200,0.005],0.01);

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

% %FREE VIBRATION-SETTLING TIME
% 
% Tsim = 10;
% t    = (0:h:Tsim)';
% 
% Tpulse = 0.05;
% v      = 2000/Tpulse;
% 
% u_step = zeros(size(t));
% u_step(t >= 1  & t < 1  + Tpulse) =  v;
% u     = u_step;
% simin = [t, u];
% 
% sim festotemplate_controller.slx
% 
% deflection          = simout2.Data(end-size(t)+1:end);
% deflection_velocity = simout3.Data(end-size(t)+1:end);
% x_laser             = -(simout1.Data - simout1.Data(1));
% x_actuator          =   simout.Data  - simout.Data(1);
% 
% %% Settling time (10% criterion, relative to t=1)
% idx_start = find(t >= 1.0, 1);
% damp_data = abs(deflection(idx_start:end));
% t_damp    = t(idx_start:end);
% threshold = 0.10 * max(damp_data);
% window    = 180;
% t_settling = NaN;
% for i = 1:(length(damp_data)-window)
%     if max(damp_data(i:i+window)) <= threshold
%         t_settling = t_damp(i) - 1.0;
%         break
%     end
% end
% 
% peak_deflection = max(abs(deflection));
% rms_deflection  = rms(deflection);
% rms_u           = rms(u);
% 
% fprintf('\n===== EXPERIMENT 1: FREE VIBRATION [Hinf] =====\n')
% fprintf('Settling time (10%% criterion):  %.3f s\n', t_settling)
% fprintf('Peak deflection:                %.4e m\n',  peak_deflection)
% fprintf('RMS deflection:                 %.4e m\n',  rms_deflection)
% fprintf('RMS control effort:             %.4e\n',    rms_u)
% 
% %% Plots - Experiment 1
% figure('Name', 'Exp1: Free Vibration [Hinf]')
% 
% subplot(3,1,1)
% plot(t, x_laser,    'b-',  'LineWidth', 0.8); hold on
% plot(t, x_actuator, 'r--', 'LineWidth', 0.8); hold off
% xlabel('Time, t [s]')
% ylabel('Position [m]')
% legend('Beam tip position, x_{laser}', 'Cart position, x_{cart}')
% grid on
% 
% subplot(3,1,2)
% plot(t, deflection, 'g-', 'LineWidth', 0.8)
% xlabel('Time, t [s]')
% ylabel('Beam deflection, \delta [m]')
% if ~isnan(t_settling)
%     xline(1.0 + t_settling, 'k--', ...
%         sprintf('t_s = %.2f s', t_settling), ...
%         'LabelVerticalAlignment', 'bottom')
% end
% grid on
% 
% subplot(3,1,3)
% plot(t, u, 'k-', 'LineWidth', 0.8)
% xlabel('Time, t [s]')
% ylabel('Control input, u [V]')
% grid on
% 
% writematrix([t, u, deflection, deflection_velocity], ...
%     'Exp1_FreeVibration_Hinf.csv')


%REFERENCE TRACKING - SQUARE WAVE VELOCITY

Tsim = 10;
t    = (0:h:Tsim)';

freq_ref = 5;   % rad/s
amp_ref  = 4;   % velocity amplitude

r_vel = amp_ref * square(freq_ref*t);

u     = r_vel;
simin = [t, u];

sim festotemplate_controller.slx

deflection2          = simout2.Data(end-length(t)+1:end);
deflection_velocity2 = simout3.Data(end-length(t)+1:end);
x_laser2             = -(simout1.Data - simout1.Data(1));
x_actuator2          =   simout.Data  - simout.Data(1);

peak_defl2 = max(abs(deflection2));
rms_defl2  = rms(deflection2);
rms_u2     = rms(u);

fprintf('\n===== EXPERIMENT 2: REFERENCE TRACKING [Hinf] =====\n')
fprintf('Peak deflection:                %.4e m\n', peak_defl2)
fprintf('RMS deflection:                 %.4e m\n', rms_defl2)
fprintf('RMS control effort:             %.4e\n',   rms_u2)

%% Plots - Experiment 2
figure('Name', 'Exp2: Reference Tracking [Hinf]')

subplot(3,1,1)
plot(t, u,          'g-',  'LineWidth', 0.8); hold on
plot(t, x_actuator2,'r-',  'LineWidth', 0.8); hold off
xlabel('Time, t [s]')
ylabel('Cart Velocity Input / Cart position  [cm/s, m]')
legend('Reference input, u', 'Cart position, x_{cart}')
grid on

subplot(3,1,2)
plot(t, deflection2, 'b-', 'LineWidth', 0.8)
xlabel('Time, t [s]')
ylabel('Beam deflection, \delta [m]')
grid on

subplot(3,1,3)
plot(t, deflection_velocity2, 'r-', 'LineWidth', 0.8)
xlabel('Time, t [s]')
ylabel('Deflection velocity, d\delta/dt [m/s]')
grid on

writematrix([t, u, x_actuator2, deflection2, deflection_velocity2], ...
    'Exp2_Tracking_Hinf.csv')

fprintf('\nDone. Data saved.\n')
