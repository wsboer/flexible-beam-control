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

L = [0.0000,  -0.0002,  0.0000;
    -0.0001,   0.0053,  0.0000;
     0.0000,   0.0000,  0.0000];


Akg = Anew - L*Cnew;
Bkg = [Bnew, L];
Ckg = eye(3);
Dkg = zeros(3, 4);

Klqr = [-4.7204e+02, -2.4792e+01, 2.4350e+01]; % Q=100,100,10 R=0.1

%FREE VIBRATION-SETTLING TIME

Tsim = 15;
t    = (0:h:Tsim)';

Tpulse = 0.05;
v      = 30/Tpulse;

u_step = zeros(size(t));
u_step(t >= 1  & t < 1  + Tpulse*0.75) =  v;
u_step(t >= 2  & t < 2  + Tpulse*0.75) = -v;
u_step(t >= 4  & t < 4  + Tpulse)      =  1.5*v;
u_step(t >= 8  & t < 8  + Tpulse)      = -v;
u_step(t >= 11 & t < 11 + Tpulse)      =  v;

u     = u_step;
simin = [t, u];

sim festotemplate.slx

deflection          = simout2.Data(end-length(t)+1:end);
deflection_velocity = simout3.Data(end-length(t)+1:end);
x_laser             = -(simout1.Data - simout1.Data(1));
x_actuator          =   simout.Data  - simout.Data(1);

%% Settling time (10% criterion, relative to t=1)
idx_start = find(t >= 1.0, 1);
damp_data = abs(deflection(idx_start:end));
t_damp    = t(idx_start:end);
threshold = 0.10 * max(damp_data);
window    = 180;
t_settling = NaN;
for i = 1:(length(damp_data)-window)
    if max(damp_data(i:i+window)) <= threshold
        t_settling = t_damp(i) - 1.0;
        break
    end
end

peak_deflection = max(abs(deflection));
rms_deflection  = rms(deflection);
rms_u           = rms(u);

fprintf('\n===== EXPERIMENT 1: FREE VIBRATION [LQG] =====\n')
fprintf('Settling time (10%% criterion):  %.3f s\n', t_settling)
fprintf('Peak deflection:                %.4e m\n',  peak_deflection)
fprintf('RMS deflection:                 %.4e m\n',  rms_deflection)
fprintf('RMS control effort:             %.4e\n',    rms_u)

%% Plots - Experiment 1
figure('Name', 'Exp1: Free Vibration [LQG]')

subplot(3,1,1)
plot(t, x_laser,    'b-',  'LineWidth', 0.8); hold on
plot(t, x_actuator, 'r--', 'LineWidth', 0.8); hold off
xlabel('Time, t [s]')
ylabel('Position [m]')
legend('Beam tip position, x_{laser}', 'Cart position, x_{cart}')
grid on

subplot(3,1,2)
plot(t, deflection, 'g-', 'LineWidth', 0.8)
xlabel('Time, t [s]')
ylabel('Beam deflection, \delta [m]')
if ~isnan(t_settling)
    xline(1.0 + t_settling, 'k--', ...
        sprintf('t_s = %.2f s', t_settling), ...
        'LabelVerticalAlignment', 'bottom')
end
grid on

subplot(3,1,3)
plot(t, u, 'k-', 'LineWidth', 0.8)
xlabel('Time, t [s]')
ylabel('Control input, u [V]')
grid on

writematrix([t, u, deflection, deflection_velocity], ...
    'Exp1_FreeVibration_LQG.csv')

%% ========================================================
%  EXPERIMENT 2: REFERENCE TRACKING (SQUARE WAVE VELOCITY)
%  Square wave velocity reference fed directly as simin
%% ========================================================

Tsim = 10;
t    = (0:h:Tsim)';

freq_ref = 5;   % rad/s
amp_ref  = 4;   % velocity amplitude - adjust on setup if needed

r_vel = amp_ref * square(freq_ref*t);

u     = r_vel;
simin = [t, u];

sim festotemplate.slx

deflection2          = simout2.Data(end-length(t)+1:end);
deflection_velocity2 = simout3.Data(end-length(t)+1:end);
x_laser2             = -(simout1.Data - simout1.Data(1));
x_actuator2          =   simout.Data  - simout.Data(1);

peak_defl2 = max(abs(deflection2));
rms_defl2  = rms(deflection2);
rms_u2     = rms(u);

fprintf('\n===== EXPERIMENT 2: REFERENCE TRACKING [LQG] =====\n')
fprintf('Peak deflection:                %.4e m\n', peak_defl2)
fprintf('RMS deflection:                 %.4e m\n', rms_defl2)
fprintf('RMS control effort:             %.4e\n',   rms_u2)

%% Plots - Experiment 2
figure('Name', 'Exp2: Reference Tracking [LQG]')

subplot(3,1,1)
plot(t, u,          'g-',  'LineWidth', 0.8); hold on
plot(t, x_actuator2,'r-',  'LineWidth', 0.8); hold off
xlabel('Time, t [s]')
ylabel('Velocity / Cart position [m/s, m]')
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
    'Exp2_Tracking_LQG.csv')

fprintf('\nDone. Data saved.\n')
