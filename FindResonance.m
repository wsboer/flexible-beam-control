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
plot(t,x_laser,t,x_actuator)
legend("Cantilever Position", "Cart Position")
subplot(1,2,2)
plot(t, deflection)

%FFT

FFT_Chirp = fft(deflection);
N = length(FFT_Chirp);
f = (0:N-1)*(Fs/N);

figure
plot(f(1:floor(N/2)), abs(FFT_Chirp(1:floor(N/2))))
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')
grid on

%PSD

figure
plot(f(1:floor(N/2)), 1/N*abs(FFT_Chirp(1:floor(N/2))).^2)
xlabel('Frequency (Hz)')
ylabel('1/N|Y(f)|^2')
grid on

