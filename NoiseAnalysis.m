clear;clc;
 
h=1e-3;
fs=1/h;
 
disturbance_data = readtable("NoiseMeasurement.csv");
 
t = disturbance_data.Var1;
u = disturbance_data.Var2;
deflection_disturbance = disturbance_data.Var3;
velocity_disturbance = disturbance_data.Var4;
 
[d_peaks,d_locs] = findpeaks(deflection_disturbance);
[v_peaks,v_locs] = findpeaks(velocity_disturbance);
 
mean_deflection = mean(abs(d_peaks))
mean_velocity = mean(abs(v_peaks))
 
deflection_variance = var(deflection_disturbance)
velocity_variance = var(velocity_disturbance)
 
%Time Domain Plots
figure
subplot(1,2,1)
plot(t, deflection_disturbance, 'b-', 'LineWidth', 0.7)
xlabel('Time, t [s]')
ylabel('Beam deflection, \delta [m]')
grid on
 
subplot(1,2,2)
plot(t, velocity_disturbance, 'r-', 'LineWidth', 0.7)
xlabel('Time, t [s]')
ylabel('Beam deflection velocity, d\delta/dt [m/s]')
grid on
 
%FFT
FFT_deflection = fft(deflection_disturbance);
N = length(FFT_deflection);
f = (0:N-1)*(fs/N);
 
figure
plot(f(1:floor(N/2)), abs(FFT_deflection(1:floor(N/2))), 'b-', 'LineWidth', 0.8)
xlabel('Frequency, f [Hz]')
ylabel('|Y(f)| [m]')
xlim([0 50])
grid on
 
%PSD
figure
plot(f(1:floor(N/2)), 1/N*abs(FFT_deflection(1:floor(N/2))).^2, 'b-', 'LineWidth', 0.8)
xlabel('Frequency, f [Hz]')
ylabel('PSD, (1/N)|Y(f)|^2 [m^2]')
xlim([0 50])
grid on

