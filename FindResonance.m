%Import Data
data = readtable('ResonanceDataChirp45.csv');

h = 1e-3;
Fs = 1/h;

t = data.Var1;
u = data.Var2;
x_laser = data.Var4;
x_actuator = data.Var3;

%Processing
%Set measurements to zero
x_laser = -(x_laser - x_laser(1));
x_actuator = x_actuator - x_actuator(1);

deflection = x_laser - x_actuator;

%Time Domain Plots
figure
subplot(1,2,1)
plot(t, x_laser, 'b-', 'LineWidth', 0.8)
hold on
plot(t, x_actuator, 'r-', 'LineWidth', 0.8)
hold off
xlabel('Time, t [s]')
ylabel('Position [m]')
legend('Cantilever tip position, x_{laser}', 'Cart position, x_{cart}')
grid on

subplot(1,2,2)
plot(t, deflection, 'g-', 'LineWidth', 0.8)
xlabel('Time, t [s]')
ylabel('Beam deflection, \delta [m]')
grid on

%FFT
FFT_Chirp = fft(deflection);
N = length(FFT_Chirp);
f = (0:N-1)*(Fs/N);

figure
plot(f(1:floor(N/2)), abs(FFT_Chirp(1:floor(N/2))), 'b-', 'LineWidth', 0.8)
xlabel('Frequency, f [Hz]')
ylabel('|Y(f)| [m]')
xlim([0 7])
grid on

%PSD
figure
plot(f(1:floor(N/2)), 1/N*abs(FFT_Chirp(1:floor(N/2))).^2, 'b-', 'LineWidth', 0.8)
xlabel('Frequency, f [Hz]')
ylabel('PSD, (1/N)|Y(f)|^2 [m^2]')
xlim([0 7])
grid on
